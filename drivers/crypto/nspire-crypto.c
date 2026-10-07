// SPDX-License-Identifier: GPL-2.0-only
/*
 * SHA-256 and (triple) DES engines of TI-Nspire calculators
 *
 * SHA-256: processes one 64 byte block per command, starting either from
 * the standard initial hash value or from its current state. The state can
 * be read but not loaded, so the engine can only compute a hash it runs from
 * start to end: one-shot digests use it, everything else (and any digest
 * while the engine is in use) runs in software.
 *
 * DES: encrypts or decrypts one 64 bit block per write with three keys
 * (EDE); single DES uses the same key three times. Keys are loaded without
 * their parity bits.
 */

#include <crypto/internal/des.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha2.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

/* SHA-256 engine */
#define SHA_CTRL		0x00
#define SHA_CTRL_BUSY		BIT(0)
#define SHA_CTRL_RESET		0x10
#define SHA_CTRL_FIRST		0x0a
#define SHA_CTRL_NEXT		0x0e
#define SHA_ACCESS		0x08
#define SHA_ACCESS_RW		BIT(8)
#define SHA_BLOCK(n)		(0x10 + 4 * (n))
#define SHA_STATE(n)		(0x60 + 4 * (n))

/* DES engine */
#define DES_BLOCK_RIGHT		0x00
#define DES_BLOCK_LEFT		0x04	/* writing it runs the engine */
#define DES_KEY_LO(n)		(0x08 + 8 * (n))
#define DES_KEY_HI(n)		(0x0c + 8 * (n))
#define DES_KEY_HI_DECRYPT	BIT(30)

/* Peripheral bus access gates in the power management block */
#define NSPIRE_PWR_BUS_DISABLE	0x18
#define NSPIRE_PWR_BUS_DES	BIT(9)
#define NSPIRE_PWR_BUS_SHA	BIT(10)

#define NSPIRE_SHA_TIMEOUT_US	1000

struct nspire_crypto_variant {
	int (*register_algs)(struct platform_device *pdev, void __iomem *base);
	void (*unregister_algs)(void);
	u32 bus_gate;
};

/* SHA-256 */

static void __iomem *nspire_sha_base;
static unsigned long nspire_sha_busy;

#define NSPIRE_SHA_STATE_SIZE	(sizeof(struct __sha256_ctx) + 1)

static int nspire_sha_block(const u8 *data, bool first)
{
	u32 ctrl;
	int i;

	for (i = 0; i < SHA256_BLOCK_SIZE / 4; i++)
		writel(get_unaligned_be32(data + 4 * i), nspire_sha_base + SHA_BLOCK(i));

	writel(first ? SHA_CTRL_FIRST : SHA_CTRL_NEXT, nspire_sha_base + SHA_CTRL);

	return readl_poll_timeout_atomic(nspire_sha_base + SHA_CTRL, ctrl,
					 !(ctrl & SHA_CTRL_BUSY), 1,
					 NSPIRE_SHA_TIMEOUT_US);
}

/* Hash a whole message with the engine; the caller owns it. */
static int nspire_sha_hw_digest(const u8 *data, unsigned int len, u8 *out)
{
	u8 tail[2 * SHA256_BLOCK_SIZE] = { };
	unsigned int full = len / SHA256_BLOCK_SIZE * SHA256_BLOCK_SIZE;
	unsigned int rest = len - full, tail_len, off;
	bool first = true;
	int i, ret;

	writel(SHA_CTRL_RESET, nspire_sha_base + SHA_CTRL);
	writel(0, nspire_sha_base + SHA_CTRL);

	for (off = 0; off < full; off += SHA256_BLOCK_SIZE, first = false) {
		ret = nspire_sha_block(data + off, first);
		if (ret)
			return ret;
	}

	/* Padding: 0x80, zeros, then the message length in bits */
	memcpy(tail, data + full, rest);
	tail[rest] = 0x80;
	tail_len = rest + 1 + 8 <= SHA256_BLOCK_SIZE ? SHA256_BLOCK_SIZE :
						      2 * SHA256_BLOCK_SIZE;
	put_unaligned_be64((u64)len << 3, tail + tail_len - 8);

	for (off = 0; off < tail_len; off += SHA256_BLOCK_SIZE, first = false) {
		ret = nspire_sha_block(tail + off, first);
		if (ret)
			return ret;
	}

	for (i = 0; i < SHA256_DIGEST_SIZE / 4; i++)
		put_unaligned_be32(readl(nspire_sha_base + SHA_STATE(i)), out + 4 * i);

	return 0;
}

#define NSPIRE_SHA_CTX(desc) ((struct sha256_ctx *)shash_desc_ctx(desc))

static int nspire_sha_init(struct shash_desc *desc)
{
	sha256_init(NSPIRE_SHA_CTX(desc));
	return 0;
}

static int nspire_sha_update(struct shash_desc *desc, const u8 *data,
			     unsigned int len)
{
	sha256_update(NSPIRE_SHA_CTX(desc), data, len);
	return 0;
}

static int nspire_sha_final(struct shash_desc *desc, u8 *out)
{
	sha256_final(NSPIRE_SHA_CTX(desc), out);
	return 0;
}

static int nspire_sha_digest(struct shash_desc *desc, const u8 *data,
			     unsigned int len, u8 *out)
{
	if (!test_and_set_bit_lock(0, &nspire_sha_busy)) {
		int ret = nspire_sha_hw_digest(data, len, out);

		clear_bit_unlock(0, &nspire_sha_busy);
		if (!ret)
			return 0;
	}

	sha256(data, len, out);
	return 0;
}

static int nspire_sha_finup(struct shash_desc *desc, const u8 *data,
			    unsigned int len, u8 *out)
{
	/* Nothing hashed yet: the engine can do the whole message */
	if (!NSPIRE_SHA_CTX(desc)->ctx.bytecount)
		return nspire_sha_digest(desc, data, len, out);

	sha256_update(NSPIRE_SHA_CTX(desc), data, len);
	sha256_final(NSPIRE_SHA_CTX(desc), out);
	return 0;
}

/* Same state format as sha256-lib */
static int nspire_sha_export(struct shash_desc *desc, void *out)
{
	struct __sha256_ctx ctx = NSPIRE_SHA_CTX(desc)->ctx;
	unsigned int partial = ctx.bytecount % SHA256_BLOCK_SIZE;
	u8 *p = out;

	ctx.bytecount -= partial;
	memcpy(p, &ctx, sizeof(ctx));
	p[sizeof(ctx)] = partial;
	return 0;
}

static int nspire_sha_import(struct shash_desc *desc, const void *in)
{
	struct __sha256_ctx *ctx = &NSPIRE_SHA_CTX(desc)->ctx;
	const u8 *p = in;

	memcpy(ctx, p, sizeof(*ctx));
	ctx->bytecount += p[sizeof(*ctx)];
	return 0;
}

static int nspire_sha_export_core(struct shash_desc *desc, void *out)
{
	memcpy(out, &NSPIRE_SHA_CTX(desc)->ctx, offsetof(struct __sha256_ctx, buf));
	return 0;
}

static int nspire_sha_import_core(struct shash_desc *desc, const void *in)
{
	memcpy(&NSPIRE_SHA_CTX(desc)->ctx, in, offsetof(struct __sha256_ctx, buf));
	return 0;
}

static struct shash_alg nspire_sha_alg = {
	.base.cra_name		= "sha256",
	.base.cra_driver_name	= "sha256-nspire",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY,
	.base.cra_blocksize	= SHA256_BLOCK_SIZE,
	.base.cra_module	= THIS_MODULE,
	.digestsize		= SHA256_DIGEST_SIZE,
	.init			= nspire_sha_init,
	.update			= nspire_sha_update,
	.final			= nspire_sha_final,
	.finup			= nspire_sha_finup,
	.digest			= nspire_sha_digest,
	.export			= nspire_sha_export,
	.import			= nspire_sha_import,
	.export_core		= nspire_sha_export_core,
	.import_core		= nspire_sha_import_core,
	.descsize		= sizeof(struct sha256_ctx),
	.statesize		= NSPIRE_SHA_STATE_SIZE,
};

static int nspire_sha_register(struct platform_device *pdev, void __iomem *base)
{
	u32 access;

	nspire_sha_base = base;

	/* Make sure the engine's registers are writable */
	access = readl(base + SHA_ACCESS);
	if (!(access & SHA_ACCESS_RW))
		writel(access | SHA_ACCESS_RW, base + SHA_ACCESS);

	return crypto_register_shash(&nspire_sha_alg);
}

static void nspire_sha_unregister(void)
{
	crypto_unregister_shash(&nspire_sha_alg);
}

/* DES */

static void __iomem *nspire_des_base;
static DEFINE_SPINLOCK(nspire_des_lock);

struct nspire_des_ctx {
	u32 key_lo[3], key_hi[3];	/* register values, without parity */
};

/* The 7 key bits of each byte, most significant byte first */
static u64 nspire_des_pack_key(const u8 *key)
{
	u64 packed = 0;
	int i;

	for (i = 0; i < DES_KEY_SIZE; i++)
		packed = packed << 7 | key[i] >> 1;

	return packed;
}

static void nspire_des_set_keys(struct nspire_des_ctx *ctx, const u8 *keys)
{
	int i;

	for (i = 0; i < 3; i++) {
		u64 packed = nspire_des_pack_key(keys + i * DES_KEY_SIZE);

		ctx->key_lo[i] = lower_32_bits(packed);
		ctx->key_hi[i] = upper_32_bits(packed);
	}
}

static int nspire_des_setkey(struct crypto_skcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
	struct nspire_des_ctx *ctx = crypto_skcipher_ctx(tfm);
	u8 keys[DES3_EDE_KEY_SIZE];
	int ret;

	ret = verify_skcipher_des_key(tfm, key);
	if (ret)
		return ret;

	/* EDE with three identical keys is plain DES */
	memcpy(keys, key, DES_KEY_SIZE);
	memcpy(keys + DES_KEY_SIZE, key, DES_KEY_SIZE);
	memcpy(keys + 2 * DES_KEY_SIZE, key, DES_KEY_SIZE);
	nspire_des_set_keys(ctx, keys);
	memzero_explicit(keys, sizeof(keys));

	return 0;
}

static int nspire_des3_setkey(struct crypto_skcipher *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct nspire_des_ctx *ctx = crypto_skcipher_ctx(tfm);
	int ret;

	ret = verify_skcipher_des3_key(tfm, key);
	if (ret)
		return ret;

	nspire_des_set_keys(ctx, key);
	return 0;
}

/* Called with nspire_des_lock held */
static void nspire_des_load_keys(const struct nspire_des_ctx *ctx, bool decrypt)
{
	int i;

	for (i = 0; i < 3; i++) {
		writel(ctx->key_lo[i], nspire_des_base + DES_KEY_LO(i));
		writel(ctx->key_hi[i] | (decrypt && !i ? DES_KEY_HI_DECRYPT : 0),
		       nspire_des_base + DES_KEY_HI(i));
	}
}

/* Called with nspire_des_lock held */
static void nspire_des_block(u8 *out, const u8 *in)
{
	writel(get_unaligned_be32(in + 4), nspire_des_base + DES_BLOCK_RIGHT);
	writel(get_unaligned_be32(in), nspire_des_base + DES_BLOCK_LEFT);
	put_unaligned_be32(readl(nspire_des_base + DES_BLOCK_LEFT), out);
	put_unaligned_be32(readl(nspire_des_base + DES_BLOCK_RIGHT), out + 4);
}

static int nspire_des_crypt(struct skcipher_request *req, bool decrypt, bool cbc)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	const struct nspire_des_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct skcipher_walk walk;
	unsigned int nbytes;
	int ret;

	ret = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes)) {
		const u8 *src = walk.src.virt.addr;
		u8 *dst = walk.dst.virt.addr;
		u8 *iv = walk.iv;
		u8 buf[DES_BLOCK_SIZE];

		spin_lock_bh(&nspire_des_lock);
		nspire_des_load_keys(ctx, decrypt);

		for (; nbytes >= DES_BLOCK_SIZE; nbytes -= DES_BLOCK_SIZE,
		     src += DES_BLOCK_SIZE, dst += DES_BLOCK_SIZE) {
			if (!cbc) {
				nspire_des_block(dst, src);
			} else if (!decrypt) {
				crypto_xor_cpy(buf, src, iv, DES_BLOCK_SIZE);
				nspire_des_block(dst, buf);
				memcpy(iv, dst, DES_BLOCK_SIZE);
			} else {
				/* src and dst may be the same */
				memcpy(buf, src, DES_BLOCK_SIZE);
				nspire_des_block(dst, src);
				crypto_xor(dst, iv, DES_BLOCK_SIZE);
				memcpy(iv, buf, DES_BLOCK_SIZE);
			}
		}

		spin_unlock_bh(&nspire_des_lock);
		memzero_explicit(buf, sizeof(buf));

		ret = skcipher_walk_done(&walk, nbytes);
	}

	return ret;
}

static int nspire_des_ecb_encrypt(struct skcipher_request *req)
{
	return nspire_des_crypt(req, false, false);
}

static int nspire_des_ecb_decrypt(struct skcipher_request *req)
{
	return nspire_des_crypt(req, true, false);
}

static int nspire_des_cbc_encrypt(struct skcipher_request *req)
{
	return nspire_des_crypt(req, false, true);
}

static int nspire_des_cbc_decrypt(struct skcipher_request *req)
{
	return nspire_des_crypt(req, true, true);
}

#define NSPIRE_DES_ALG(_name, _drv, _keysize, _setkey, _ivsize, _mode) {	\
	.base.cra_name		= _name,					\
	.base.cra_driver_name	= _drv,						\
	.base.cra_priority	= 300,						\
	.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY,			\
	.base.cra_blocksize	= DES_BLOCK_SIZE,				\
	.base.cra_ctxsize	= sizeof(struct nspire_des_ctx),		\
	.base.cra_module	= THIS_MODULE,					\
	.min_keysize		= _keysize,					\
	.max_keysize		= _keysize,					\
	.ivsize			= _ivsize,					\
	.setkey			= _setkey,					\
	.encrypt		= nspire_des_##_mode##_encrypt,			\
	.decrypt		= nspire_des_##_mode##_decrypt,			\
}

static struct skcipher_alg nspire_des_algs[] = {
	NSPIRE_DES_ALG("ecb(des)", "ecb-des-nspire", DES_KEY_SIZE,
		       nspire_des_setkey, 0, ecb),
	NSPIRE_DES_ALG("cbc(des)", "cbc-des-nspire", DES_KEY_SIZE,
		       nspire_des_setkey, DES_BLOCK_SIZE, cbc),
	NSPIRE_DES_ALG("ecb(des3_ede)", "ecb-des3-nspire", DES3_EDE_KEY_SIZE,
		       nspire_des3_setkey, 0, ecb),
	NSPIRE_DES_ALG("cbc(des3_ede)", "cbc-des3-nspire", DES3_EDE_KEY_SIZE,
		       nspire_des3_setkey, DES3_EDE_BLOCK_SIZE, cbc),
};

static int nspire_des_register(struct platform_device *pdev, void __iomem *base)
{
	nspire_des_base = base;
	return crypto_register_skciphers(nspire_des_algs, ARRAY_SIZE(nspire_des_algs));
}

static void nspire_des_unregister(void)
{
	crypto_unregister_skciphers(nspire_des_algs, ARRAY_SIZE(nspire_des_algs));
}

/* Platform glue */

static int nspire_crypto_probe(struct platform_device *pdev)
{
	const struct nspire_crypto_variant *variant;
	void __iomem *base;
	struct regmap *pwr;

	variant = device_get_match_data(&pdev->dev);
	if (!variant)
		return -ENODEV;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/* The OS may have cut the engine off the bus */
	pwr = syscon_regmap_lookup_by_compatible("ti,nspire-pwr");
	if (!IS_ERR(pwr))
		regmap_clear_bits(pwr, NSPIRE_PWR_BUS_DISABLE, variant->bus_gate);

	return variant->register_algs(pdev, base);
}

static void nspire_crypto_remove(struct platform_device *pdev)
{
	const struct nspire_crypto_variant *variant = device_get_match_data(&pdev->dev);

	variant->unregister_algs();
}

static const struct nspire_crypto_variant nspire_sha_variant = {
	.register_algs		= nspire_sha_register,
	.unregister_algs	= nspire_sha_unregister,
	.bus_gate		= NSPIRE_PWR_BUS_SHA,
};

static const struct nspire_crypto_variant nspire_des_variant = {
	.register_algs		= nspire_des_register,
	.unregister_algs	= nspire_des_unregister,
	.bus_gate		= NSPIRE_PWR_BUS_DES,
};

static const struct of_device_id nspire_crypto_of_match[] = {
	{ .compatible = "ti,nspire-sha256", .data = &nspire_sha_variant },
	{ .compatible = "ti,nspire-des", .data = &nspire_des_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, nspire_crypto_of_match);

static struct platform_driver nspire_crypto_driver = {
	.probe	= nspire_crypto_probe,
	.remove	= nspire_crypto_remove,
	.driver	= {
		.name		= "nspire-crypto",
		.of_match_table	= nspire_crypto_of_match,
	},
};
module_platform_driver(nspire_crypto_driver);

MODULE_DESCRIPTION("TI-Nspire SHA-256 and DES engine driver");
MODULE_LICENSE("GPL");
