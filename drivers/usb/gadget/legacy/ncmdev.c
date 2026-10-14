// SPDX-License-Identifier: GPL-2.0+
/*
 * ncmdev: a CDC NCM gadget for a network stack in user space
 *
 * The host sees a CDC NCM network adapter. The kernel only answers the
 * NCM control requests and tells the host when the link is up; the NCM
 * transfer blocks (NTBs) go through /dev/ncm as they are:
 *
 *	read()	one NTB the host sent (up to NTB_SIZE bytes)
 *	write()	one NTB to send (up to what the host accepts)
 *	poll()	readable when an NTB came in, writable when one can go out
 *
 * Building and parsing the NTBs (16-bit format only) and the network
 * stack itself are up to user space, so the kernel needs no networking.
 * The descriptors and control requests follow f_ncm.c.
 */
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/usb/cdc.h>
#include <linux/usb/composite.h>

#define DRIVER_DESC	"NCM device"

/* the IDs of the Linux NCM gadget, g_ncm: the host's driver is the same */
#define CDC_VENDOR_NUM	0x0525	/* NetChip */
#define CDC_PRODUCT_NUM	0xa4a1	/* Linux-USB Ethernet Gadget */

#define NCMDEV_MINOR	241	/* misc, local/experimental: a static node */

/* NTBs both ways: the smallest the NCM specification allows */
#define NTB_SIZE	2048
#define NR_RX		2
#define NR_TX		2
#define MAX_SEGMENT	1514	/* an Ethernet frame without its CRC */

#define STATUS_BYTECOUNT	16	/* 8 byte header + data */
#define STATUS_INTERVAL_MS	32

USB_GADGET_COMPOSITE_OPTIONS();

static char *host_addr = "02fc00000001";
module_param(host_addr, charp, 0444);
MODULE_PARM_DESC(host_addr, "the host's Ethernet address, 12 hex digits");

/* ------------------------------------------------------------ descriptors */

static struct usb_cdc_ncm_ntb_parameters ntb_parameters = {
	.wLength = cpu_to_le16(sizeof(ntb_parameters)),
	.bmNtbFormatsSupported = cpu_to_le16(USB_CDC_NCM_NTB16_SUPPORTED),
	.dwNtbInMaxSize = cpu_to_le32(NTB_SIZE),
	.wNdpInDivisor = cpu_to_le16(4),
	.wNdpInPayloadRemainder = cpu_to_le16(0),
	.wNdpInAlignment = cpu_to_le16(4),
	.dwNtbOutMaxSize = cpu_to_le32(NTB_SIZE),
	.wNdpOutDivisor = cpu_to_le16(4),
	.wNdpOutPayloadRemainder = cpu_to_le16(0),
	.wNdpOutAlignment = cpu_to_le16(4),
};

static struct usb_interface_assoc_descriptor iad_desc = {
	.bLength =		sizeof(iad_desc),
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,
	.bInterfaceCount =	2,	/* control + data */
	.bFunctionClass =	USB_CLASS_COMM,
	.bFunctionSubClass =	USB_CDC_SUBCLASS_NCM,
	.bFunctionProtocol =	USB_CDC_PROTO_NONE,
};

static struct usb_interface_descriptor control_intf = {
	.bLength =		sizeof(control_intf),
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_NCM,
	.bInterfaceProtocol =	USB_CDC_PROTO_NONE,
};

static struct usb_cdc_header_desc header_desc = {
	.bLength =		sizeof(header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_union_desc union_desc = {
	.bLength =		sizeof(union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
};

static struct usb_cdc_ether_desc ether_desc = {
	.bLength =		sizeof(ether_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ETHERNET_TYPE,
	.bmEthernetStatistics =	cpu_to_le32(0),
	.wMaxSegmentSize =	cpu_to_le16(MAX_SEGMENT),
	.wNumberMCFilters =	cpu_to_le16(0),
	.bNumberPowerFilters =	0,
};

static struct usb_cdc_ncm_desc ncm_desc = {
	.bLength =		sizeof(ncm_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_NCM_TYPE,
	.bcdNcmVersion =	cpu_to_le16(0x0100),
	.bmNetworkCapabilities = USB_CDC_NCM_NCAP_ETH_FILTER,
};

/* the data interface: no endpoints in altsetting 0, two bulk ones in 1 */
static struct usb_interface_descriptor data_nop_intf = {
	.bLength =		sizeof(data_nop_intf),
	.bDescriptorType =	USB_DT_INTERFACE,
	.bAlternateSetting =	0,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceProtocol =	USB_CDC_NCM_PROTO_NTB,
};

static struct usb_interface_descriptor data_intf = {
	.bLength =		sizeof(data_intf),
	.bDescriptorType =	USB_DT_INTERFACE,
	.bAlternateSetting =	1,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceProtocol =	USB_CDC_NCM_PROTO_NTB,
};

static struct usb_endpoint_descriptor fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		STATUS_INTERVAL_MS,
};

static struct usb_endpoint_descriptor fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_function[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&control_intf,
	(struct usb_descriptor_header *)&header_desc,
	(struct usb_descriptor_header *)&union_desc,
	(struct usb_descriptor_header *)&ether_desc,
	(struct usb_descriptor_header *)&ncm_desc,
	(struct usb_descriptor_header *)&fs_notify_desc,
	(struct usb_descriptor_header *)&data_nop_intf,
	(struct usb_descriptor_header *)&data_intf,
	(struct usb_descriptor_header *)&fs_in_desc,
	(struct usb_descriptor_header *)&fs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor hs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		USB_MS_TO_HS_INTERVAL(STATUS_INTERVAL_MS),
};

static struct usb_endpoint_descriptor hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *hs_function[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&control_intf,
	(struct usb_descriptor_header *)&header_desc,
	(struct usb_descriptor_header *)&union_desc,
	(struct usb_descriptor_header *)&ether_desc,
	(struct usb_descriptor_header *)&ncm_desc,
	(struct usb_descriptor_header *)&hs_notify_desc,
	(struct usb_descriptor_header *)&data_nop_intf,
	(struct usb_descriptor_header *)&data_intf,
	(struct usb_descriptor_header *)&hs_in_desc,
	(struct usb_descriptor_header *)&hs_out_desc,
	NULL,
};

#define STRING_CTRL_IDX	0
#define STRING_MAC_IDX	1
#define STRING_DATA_IDX	2

static char host_mac[13];

static struct usb_string ncm_string_defs[] = {
	[STRING_CTRL_IDX].s = "CDC Network Control Model (NCM)",
	[STRING_MAC_IDX].s = host_mac,
	[STRING_DATA_IDX].s = "CDC Network Data",
	{ }
};

static struct usb_gadget_strings ncm_string_table = {
	.language =	0x0409,	/* en-us */
	.strings =	ncm_string_defs,
};

static struct usb_gadget_strings *ncm_strings[] = {
	&ncm_string_table,
	NULL,
};

/* ---------------------------------------------------------------- state */

static struct ncmdev {
	struct usb_function	func;
	struct usb_ep		*in, *out, *notify;
	struct usb_request	*notify_req;
	struct usb_request	*rx[NR_RX], *tx[NR_TX];
	u8			ctrl_id, data_id;

	spinlock_t		lock;
	bool			active;		/* data altsetting 1 */
	bool			rx_done[NR_RX];	/* complete, not read yet */
	bool			tx_busy[NR_TX];
	unsigned int		in_size;	/* NTB size the host accepts */
	enum { NOTIFY_NONE, NOTIFY_CONNECT, NOTIFY_SPEED } notify_state;
	bool			notify_busy;
	wait_queue_head_t	wait;
	struct mutex		io_lock;	/* one reader/writer at a time */
	atomic_t		opened;
} ncm;

static unsigned int bitrate(struct usb_gadget *g)
{
	if (g->speed == USB_SPEED_HIGH)
		return 13 * 512 * 8 * 1000 * 8;
	return 19 * 64 * 1 * 1000 * 8;
}

/* Under ncm.lock */
static void do_notify(void)
{
	struct usb_request *req = ncm.notify_req;
	struct usb_cdc_notification *event = req->buf;
	__le32 *data;

	if (ncm.notify_busy)
		return;
	switch (ncm.notify_state) {
	case NOTIFY_NONE:
		return;
	case NOTIFY_CONNECT:
		event->bNotificationType = USB_CDC_NOTIFY_NETWORK_CONNECTION;
		event->wValue = cpu_to_le16(ncm.active);
		event->wLength = 0;
		req->length = sizeof(*event);
		ncm.notify_state = NOTIFY_NONE;
		break;
	case NOTIFY_SPEED:
		event->bNotificationType = USB_CDC_NOTIFY_SPEED_CHANGE;
		event->wValue = 0;
		event->wLength = cpu_to_le16(8);
		req->length = STATUS_BYTECOUNT;
		data = req->buf + sizeof(*event);
		data[0] = cpu_to_le32(bitrate(ncm.func.config->cdev->gadget));
		data[1] = data[0];
		ncm.notify_state = NOTIFY_CONNECT;
		break;
	}
	event->bmRequestType = 0xa1;
	event->wIndex = cpu_to_le16(ncm.ctrl_id);
	ncm.notify_busy = true;
	spin_unlock(&ncm.lock);
	if (usb_ep_queue(ncm.notify, req, GFP_ATOMIC) < 0) {
		spin_lock(&ncm.lock);
		ncm.notify_busy = false;
		return;
	}
	spin_lock(&ncm.lock);
}

static void notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	spin_lock(&ncm.lock);
	ncm.notify_busy = false;
	if (req->status == -ECONNRESET || req->status == -ESHUTDOWN)
		ncm.notify_state = NOTIFY_NONE;
	do_notify();
	spin_unlock(&ncm.lock);
}

static void rx_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned long flags;
	int i = (long)req->context;

	spin_lock_irqsave(&ncm.lock, flags);
	if (req->status == 0 && req->actual)
		ncm.rx_done[i] = true;
	else if (ncm.active && req->status != -ESHUTDOWN &&
		 req->status != -ECONNRESET)
		usb_ep_queue(ep, req, GFP_ATOMIC);	/* try again */
	spin_unlock_irqrestore(&ncm.lock, flags);
	wake_up_interruptible(&ncm.wait);
}

static void tx_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ncm.lock, flags);
	ncm.tx_busy[(long)req->context] = false;
	spin_unlock_irqrestore(&ncm.lock, flags);
	wake_up_interruptible(&ncm.wait);
}

/* ------------------------------------------------------------- function */

static void ep0out_complete(struct usb_ep *ep, struct usb_request *req)
{
	/* SET_NTB_INPUT_SIZE only */
	unsigned int size;

	if (req->status || req->actual != req->length)
		goto invalid;
	size = get_unaligned_le32(req->buf);
	if (size < USB_CDC_NCM_NTB_MIN_IN_SIZE)
		goto invalid;
	ncm.in_size = min(size, (unsigned int)NTB_SIZE);
	return;
invalid:
	usb_ep_set_halt(ep);
}

static int ncm_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request *req = cdev->req;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	int value = -EOPNOTSUPP;

	if (w_index != ncm.ctrl_id)
		return value;

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SET_ETHERNET_PACKET_FILTER:
		/* everything goes to user space, which filters */
		if (w_length == 0)
			value = 0;
		break;
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_NTB_PARAMETERS:
		if (w_length == 0 || w_value != 0)
			break;
		value = min_t(int, w_length, sizeof(ntb_parameters));
		memcpy(req->buf, &ntb_parameters, value);
		break;
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_NTB_INPUT_SIZE:
		if (w_length < 4 || w_value != 0)
			break;
		put_unaligned_le32(ncm.in_size, req->buf);
		value = 4;
		break;
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SET_NTB_INPUT_SIZE:
		if (w_length != 4 || w_value != 0)
			break;
		req->complete = ep0out_complete;
		value = 4;
		break;
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_NTB_FORMAT:
		if (w_length < 2 || w_value != 0)
			break;
		put_unaligned_le16(0, req->buf);	/* NTB16 */
		value = 2;
		break;
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SET_NTB_FORMAT:
		if (w_length == 0 && w_value == 0)
			value = 0;
		break;
	}

	if (value >= 0) {
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}
	return value;
}

/* Under ncm.lock: queue the receive requests that are not waiting to be read. */
static void rx_queue_all(void)
{
	int i;

	for (i = 0; i < NR_RX; i++)
		if (!ncm.rx_done[i]) {
			ncm.rx[i]->length = NTB_SIZE;
			usb_ep_queue(ncm.out, ncm.rx[i], GFP_ATOMIC);
		}
}

static void data_stop(void)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ncm.lock, flags);
	ncm.active = false;
	spin_unlock_irqrestore(&ncm.lock, flags);
	usb_ep_disable(ncm.in);
	usb_ep_disable(ncm.out);
	spin_lock_irqsave(&ncm.lock, flags);
	for (i = 0; i < NR_RX; i++)
		ncm.rx_done[i] = false;
	spin_unlock_irqrestore(&ncm.lock, flags);
	wake_up_interruptible(&ncm.wait);
}

static int ncm_set_alt(struct usb_function *f, unsigned int intf, unsigned int alt)
{
	struct usb_gadget *g = f->config->cdev->gadget;
	unsigned long flags;

	if (intf == ncm.ctrl_id) {
		if (alt)
			return -EINVAL;
		usb_ep_disable(ncm.notify);
		if (!ncm.notify->desc && config_ep_by_speed(g, f, ncm.notify))
			return -EINVAL;
		return usb_ep_enable(ncm.notify);
	}
	if (intf != ncm.data_id || alt > 1)
		return -EINVAL;

	if (ncm.active)
		data_stop();
	ncm.in_size = NTB_SIZE;
	if (alt == 1) {
		if (config_ep_by_speed(g, f, ncm.in) ||
		    config_ep_by_speed(g, f, ncm.out) ||
		    usb_ep_enable(ncm.in) || usb_ep_enable(ncm.out)) {
			usb_ep_disable(ncm.in);
			return -EINVAL;
		}
		spin_lock_irqsave(&ncm.lock, flags);
		ncm.active = true;
		rx_queue_all();
		spin_unlock_irqrestore(&ncm.lock, flags);
		wake_up_interruptible(&ncm.wait);
	}
	spin_lock_irqsave(&ncm.lock, flags);
	ncm.notify_state = NOTIFY_SPEED;
	do_notify();
	spin_unlock_irqrestore(&ncm.lock, flags);
	return 0;
}

static int ncm_get_alt(struct usb_function *f, unsigned int intf)
{
	return intf == ncm.data_id && ncm.active;
}

static void ncm_disable(struct usb_function *f)
{
	if (ncm.active)
		data_stop();
	usb_ep_disable(ncm.notify);
	ncm.notify->desc = NULL;
}

static struct usb_request *alloc_req(struct usb_ep *ep, unsigned int len,
				     void (*complete)(struct usb_ep *, struct usb_request *),
				     long i)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);

	if (!req)
		return NULL;
	req->buf = kmalloc(len, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}
	req->complete = complete;
	req->context = (void *)i;
	return req;
}

static int ncm_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct usb_string *us;
	int id, i;

	us = usb_gstrings_attach(cdev, ncm_strings, ARRAY_SIZE(ncm_string_defs));
	if (IS_ERR(us))
		return PTR_ERR(us);
	control_intf.iInterface = us[STRING_CTRL_IDX].id;
	data_nop_intf.iInterface = us[STRING_DATA_IDX].id;
	data_intf.iInterface = us[STRING_DATA_IDX].id;
	ether_desc.iMACAddress = us[STRING_MAC_IDX].id;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	ncm.ctrl_id = id;
	iad_desc.bFirstInterface = id;
	control_intf.bInterfaceNumber = id;
	union_desc.bMasterInterface0 = id;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	ncm.data_id = id;
	data_nop_intf.bInterfaceNumber = id;
	data_intf.bInterfaceNumber = id;
	union_desc.bSlaveInterface0 = id;

	ncm.in = usb_ep_autoconfig(cdev->gadget, &fs_in_desc);
	ncm.out = usb_ep_autoconfig(cdev->gadget, &fs_out_desc);
	ncm.notify = usb_ep_autoconfig(cdev->gadget, &fs_notify_desc);
	if (!ncm.in || !ncm.out || !ncm.notify)
		return -ENODEV;
	hs_in_desc.bEndpointAddress = fs_in_desc.bEndpointAddress;
	hs_out_desc.bEndpointAddress = fs_out_desc.bEndpointAddress;
	hs_notify_desc.bEndpointAddress = fs_notify_desc.bEndpointAddress;

	ncm.notify_req = alloc_req(ncm.notify, STATUS_BYTECOUNT, notify_complete, 0);
	if (!ncm.notify_req)
		return -ENOMEM;
	for (i = 0; i < NR_RX; i++) {
		ncm.rx[i] = alloc_req(ncm.out, NTB_SIZE, rx_complete, i);
		if (!ncm.rx[i])
			return -ENOMEM;
	}
	for (i = 0; i < NR_TX; i++) {
		ncm.tx[i] = alloc_req(ncm.in, NTB_SIZE, tx_complete, i);
		if (!ncm.tx[i])
			return -ENOMEM;
	}
	return usb_assign_descriptors(f, fs_function, hs_function, NULL, NULL);
}

/* --------------------------------------------------------------- /dev/ncm */

static int ncm_open(struct inode *inode, struct file *file)
{
	if (atomic_xchg(&ncm.opened, 1))
		return -EBUSY;
	return nonseekable_open(inode, file);
}

static int ncm_release(struct inode *inode, struct file *file)
{
	atomic_set(&ncm.opened, 0);
	return 0;
}

static int rx_ready(void)
{
	int i;

	for (i = 0; i < NR_RX; i++)
		if (ncm.rx_done[i])
			return i;
	return -1;
}

static int tx_free(void)
{
	int i;

	if (!ncm.active)
		return -1;
	for (i = 0; i < NR_TX; i++)
		if (!ncm.tx_busy[i])
			return i;
	return -1;
}

static ssize_t ncm_read(struct file *file, char __user *buf, size_t len,
			loff_t *ppos)
{
	struct usb_request *req;
	unsigned long flags;
	ssize_t ret;
	int i;

	if (mutex_lock_interruptible(&ncm.io_lock))
		return -ERESTARTSYS;
	for (;;) {
		spin_lock_irqsave(&ncm.lock, flags);
		i = rx_ready();
		spin_unlock_irqrestore(&ncm.lock, flags);
		if (i >= 0)
			break;
		ret = -EAGAIN;
		if (file->f_flags & O_NONBLOCK)
			goto out;
		ret = -ERESTARTSYS;
		if (wait_event_interruptible(ncm.wait, rx_ready() >= 0))
			goto out;
	}
	req = ncm.rx[i];
	ret = min_t(size_t, len, req->actual);
	if (copy_to_user(buf, req->buf, ret))
		ret = -EFAULT;
	spin_lock_irqsave(&ncm.lock, flags);
	ncm.rx_done[i] = false;
	if (ncm.active) {
		req->length = NTB_SIZE;
		usb_ep_queue(ncm.out, req, GFP_ATOMIC);
	}
	spin_unlock_irqrestore(&ncm.lock, flags);
out:
	mutex_unlock(&ncm.io_lock);
	return ret;
}

static ssize_t ncm_write(struct file *file, const char __user *buf,
			 size_t len, loff_t *ppos)
{
	struct usb_request *req;
	unsigned long flags;
	ssize_t ret;
	int i;

	if (len == 0)
		return 0;
	if (mutex_lock_interruptible(&ncm.io_lock))
		return -ERESTARTSYS;
	for (;;) {
		spin_lock_irqsave(&ncm.lock, flags);
		i = tx_free();
		if (i >= 0)
			ncm.tx_busy[i] = true;
		spin_unlock_irqrestore(&ncm.lock, flags);
		if (i >= 0)
			break;
		ret = -EAGAIN;
		if (file->f_flags & O_NONBLOCK)
			goto out;
		ret = -ERESTARTSYS;
		if (wait_event_interruptible(ncm.wait, tx_free() >= 0))
			goto out;
	}
	req = ncm.tx[i];
	ret = -EMSGSIZE;
	if (len > ncm.in_size)
		goto put_back;
	ret = -EFAULT;
	if (copy_from_user(req->buf, buf, len))
		goto put_back;
	req->length = len;
	/* a short NTB ends with a short packet */
	req->zero = len < ncm.in_size;
	ret = usb_ep_queue(ncm.in, req, GFP_KERNEL);
	if (ret == 0) {
		ret = len;
		goto out;
	}
put_back:
	spin_lock_irqsave(&ncm.lock, flags);
	ncm.tx_busy[i] = false;
	spin_unlock_irqrestore(&ncm.lock, flags);
out:
	mutex_unlock(&ncm.io_lock);
	return ret;
}

static __poll_t ncm_poll(struct file *file, poll_table *wait)
{
	unsigned long flags;
	__poll_t mask = 0;

	poll_wait(file, &ncm.wait, wait);
	spin_lock_irqsave(&ncm.lock, flags);
	if (rx_ready() >= 0)
		mask |= EPOLLIN | EPOLLRDNORM;
	if (tx_free() >= 0)
		mask |= EPOLLOUT | EPOLLWRNORM;
	spin_unlock_irqrestore(&ncm.lock, flags);
	return mask;
}

static const struct file_operations ncm_fops = {
	.owner		= THIS_MODULE,
	.open		= ncm_open,
	.release	= ncm_release,
	.read		= ncm_read,
	.write		= ncm_write,
	.poll		= ncm_poll,
};

static struct miscdevice ncm_misc = {
	.minor	= NCMDEV_MINOR,
	.name	= "ncm",
	.fops	= &ncm_fops,
	.mode	= 0600,
};

/* -------------------------------------------------------------- gadget */

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof(device_desc),
	.bDescriptorType =	USB_DT_DEVICE,
	.bDeviceClass =		USB_CLASS_MISC,	/* with an IAD */
	.bDeviceSubClass =	2,
	.bDeviceProtocol =	1,
	.idVendor =		cpu_to_le16(CDC_VENDOR_NUM),
	.idProduct =		cpu_to_le16(CDC_PRODUCT_NUM),
	.bNumConfigurations =	1,
};

static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = "Linux",
	[USB_GADGET_PRODUCT_IDX].s = DRIVER_DESC,
	[USB_GADGET_SERIAL_IDX].s = "",
	{ }
};

static struct usb_gadget_strings stringtab_dev = {
	.language =	0x0409,
	.strings =	strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static int ncm_do_config(struct usb_configuration *c)
{
	ncm.func.name = "ncm";
	ncm.func.bind = ncm_bind;
	ncm.func.set_alt = ncm_set_alt;
	ncm.func.get_alt = ncm_get_alt;
	ncm.func.setup = ncm_setup;
	ncm.func.disable = ncm_disable;
	return usb_add_function(c, &ncm.func);
}

static struct usb_configuration ncm_config = {
	.label =		"CDC Ethernet (NCM)",
	.bConfigurationValue =	1,
	.bmAttributes =		USB_CONFIG_ATT_SELFPOWER,
};

static int ncmdev_bind(struct usb_composite_dev *cdev)
{
	int status, i;

	/* the MAC address string: 12 upper case hex digits */
	for (i = 0; i < 12; i++)
		host_mac[i] = toupper(host_addr[i] ? host_addr[i] : '0');
	host_mac[12] = 0;

	status = usb_string_ids_tab(cdev, strings_dev);
	if (status < 0)
		return status;
	device_desc.iManufacturer = strings_dev[USB_GADGET_MANUFACTURER_IDX].id;
	device_desc.iProduct = strings_dev[USB_GADGET_PRODUCT_IDX].id;

	status = usb_add_config(cdev, &ncm_config, ncm_do_config);
	if (status < 0)
		return status;
	usb_composite_overwrite_options(cdev, &coverwrite);
	return 0;
}

static struct usb_composite_driver ncmdev_driver = {
	.name		= "g_ncmdev",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.max_speed	= USB_SPEED_HIGH,
	.bind		= ncmdev_bind,
};

static int __init ncmdev_init(void)
{
	int ret;

	spin_lock_init(&ncm.lock);
	init_waitqueue_head(&ncm.wait);
	mutex_init(&ncm.io_lock);
	ncm.in_size = NTB_SIZE;
	ret = misc_register(&ncm_misc);
	if (ret)
		return ret;
	ret = usb_composite_probe(&ncmdev_driver);
	if (ret)
		misc_deregister(&ncm_misc);
	return ret;
}
module_init(ncmdev_init);

static void __exit ncmdev_exit(void)
{
	usb_composite_unregister(&ncmdev_driver);
	misc_deregister(&ncm_misc);
}
module_exit(ncmdev_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
