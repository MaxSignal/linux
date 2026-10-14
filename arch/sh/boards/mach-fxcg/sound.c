// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG: tones on the 2.5 mm serial jack (the "3-pin port")
 *
 * The transmit line of the serial port is used as a plain output
 * (port J bit 2), toggled at twice the tone's frequency by channel 2 of
 * the TMU, which the clock code leaves alone. An earphone on the jack
 * plays the tones on one side.
 *
 * The port's output gate is port U bit 3, and bit 2 must be set too
 * (Simon Lothar's fx-CG20 register notes, "Accessing the 3pin serial
 * lines directly"). The pins are given back as they were after a tone.
 */
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <mach/fxcg.h>

#define PJCR		((void __iomem *)0xa4050110)	/* 2 bits per pin */
#define PJDR		((void __iomem *)0xa4050130)
#define PUCR		((void __iomem *)0xa4050142)
#define PUDR		((void __iomem *)0xa4050162)
#define TX_BIT		2				/* port J */
#define GATE_BITS	0x0c				/* port U bits 3, 2 */

#define TSTR		((void __iomem *)0xa4490004)
#define TCOR2		((void __iomem *)0xa4490020)
#define TCNT2		((void __iomem *)0xa4490024)
#define TCR2		((void __iomem *)0xa4490028)
#define TCR_UNF		(1 << 8)
#define TCR_UNIE	(1 << 5)
#define TSTR_STR2	(1 << 2)
#define TMU2_IRQ	evt2irq(0x440)

static DEFINE_SPINLOCK(snd_lock);
static struct timer_list snd_stop_timer;
static unsigned long tmu_rate;		/* the TMU counts at P/4 */
static bool playing;
static u16 saved_pjcr, saved_pucr;
static u8 saved_pudr;

static irqreturn_t snd_interrupt(int irq, void *dev_id)
{
	__raw_writew(TCR_UNIE, TCR2);		/* clears UNF */
	__raw_writeb(__raw_readb(PJDR) ^ (1 << TX_BIT), PJDR);
	return IRQ_HANDLED;
}

/* Under snd_lock */
static void snd_off(void)
{
	if (!playing)
		return;
	__raw_writeb(__raw_readb(TSTR) & ~TSTR_STR2, TSTR);
	__raw_writew(0, TCR2);
	__raw_writew(saved_pjcr, PJCR);
	__raw_writeb((__raw_readb(PUDR) & ~GATE_BITS) | (saved_pudr & GATE_BITS), PUDR);
	__raw_writew(saved_pucr, PUCR);
	playing = false;
}

/* Under snd_lock */
static void snd_on(unsigned int hz)
{
	u32 count = tmu_rate / (2 * hz);

	if (!playing) {
		saved_pjcr = __raw_readw(PJCR);
		saved_pucr = __raw_readw(PUCR);
		saved_pudr = __raw_readb(PUDR);
		/* port U bit 3 an output, then the gate open */
		__raw_writew((saved_pucr & ~(3 << 6)) | (1 << 6), PUCR);
		__raw_writeb(saved_pudr | GATE_BITS, PUDR);
		/* the transmit pin an output */
		__raw_writew((saved_pjcr & ~(3 << 2 * TX_BIT)) | (1 << 2 * TX_BIT), PJCR);
	}
	__raw_writeb(__raw_readb(TSTR) & ~TSTR_STR2, TSTR);
	__raw_writel(count, TCOR2);
	__raw_writel(count, TCNT2);
	__raw_writew(TCR_UNIE, TCR2);		/* P/4, interrupt on underflow */
	__raw_writeb(__raw_readb(TSTR) | TSTR_STR2, TSTR);
	playing = true;
}

/*
 * A tone of hz for ms milliseconds (0: until the next call), or silence
 * (hz 0). Callable from any context.
 */
void fxcg_tone(unsigned int hz, unsigned int ms)
{
	unsigned long flags;

	if (!tmu_rate)
		return;
	spin_lock_irqsave(&snd_lock, flags);
	if (hz >= 20 && hz <= 20000)
		snd_on(hz);
	else
		snd_off();
	spin_unlock_irqrestore(&snd_lock, flags);
	if (hz && ms)
		mod_timer(&snd_stop_timer, jiffies + msecs_to_jiffies(ms));
	else
		timer_delete(&snd_stop_timer);
}

static void snd_stop(struct timer_list *t)
{
	unsigned long flags;

	spin_lock_irqsave(&snd_lock, flags);
	snd_off();
	spin_unlock_irqrestore(&snd_lock, flags);
}

static int __init fxcg_sound_init(void)
{
	struct clk *clk = clk_get(NULL, "peripheral_clk");

	if (IS_ERR(clk))
		return PTR_ERR(clk);
	timer_setup(&snd_stop_timer, snd_stop, 0);
	if (request_irq(TMU2_IRQ, snd_interrupt, 0, "tone", NULL))
		return -EBUSY;
	tmu_rate = clk_get_rate(clk) / 4;
	clk_put(clk);
	return 0;
}
device_initcall(fxcg_sound_init);
