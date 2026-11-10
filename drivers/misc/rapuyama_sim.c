// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E5 (RapuYama): SIM bridge for the userspace ISASW runtime.
 *
 * The runtime (linux/e5-cell/e5rt) runs the modem firmware's original SIM HAL
 * in userspace. It traps the HAL's accesses to the SIM controller and sends
 * them here, together with the Gazoo PMIC accesses and the SIM interrupts.
 * The interface is upstream's linux/isi-runtime/sim_bridge.h (/dev/rapuyama-sim):
 *   ISI_SIM_REG   1/2/4-byte aligned access to the 0x30-byte register block
 *                 at 0x20038300 (firmware va 0xc6038300); 0x20 (data FIFO) bytes only
 *   ISI_SIM_PMIC  Gazoo read; write only to an allowlist (SIM supply/control)
 *   ISI_SIM_IRQ   enable/disable SIM interrupt 0xb1..0xb6 (INTC lines
 *                 177..182, from the E5 firmware's irq_register calls)
 *   ISI_SIM_ADC   Gazoo ADC channel 0..18 (Linux owns the ADC)
 *   read()        4 bytes: pending mask, bit n = interrupt 0xb1 + n. The
 *                 interrupts are one-shot: a line is disabled when it fires
 *                 and the runtime enables it again after its handler.
 * One opener (CAP_SYS_RAWIO). Closing disables the interrupts. It does not
 * power the SIM by itself and does nothing until the runtime asks.
 */
#include <linux/atomic.h>
#include <linux/capability.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/soc/rapuyama-pmic.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

struct isi_sim_reg { __u32 offset, size, value, write; };
struct isi_sim_pmic { __u32 device, reg, value, write; };
struct isi_sim_irq { __u32 number, enable; };
struct isi_sim_adc { __u32 channel, value; };
#define ISI_SIM_REG	_IOWR('S', 0, struct isi_sim_reg)
#define ISI_SIM_PMIC	_IOWR('S', 1, struct isi_sim_pmic)
#define ISI_SIM_IRQ	_IOW('S', 2, struct isi_sim_irq)
#define ISI_SIM_ADC	_IOWR('S', 3, struct isi_sim_adc)

#define SIM_REGS	0x30
#define SIM_NIRQ	6
#define SIM_IRQ_FIRST	0xb1

/*
 * PMIC writes the SIM path may make: device 1 regs 0x1c..0x1e (SIM supply /
 * interface, upstream's simulated peer starts them at 0x488/2/2) and device 2
 * regs 0x01/0x03 (Gazoo SIM interrupt status/mask). Everything else is refused
 * and logged, so the list can only grow from what the firmware really asks for.
 */
static const struct { u8 dev, reg; } sim_pmic_writes[] = {
	{ 1, 0x1c }, { 1, 0x1d }, { 1, 0x1e }, { 2, 0x01 }, { 2, 0x03 },
};

struct sim_bridge {
	void __iomem *regs;
	int irq[SIM_NIRQ];
	bool enabled[SIM_NIRQ];
	u32 pending;
	spinlock_t lock;
	wait_queue_head_t wait;
	atomic_t open;
	struct miscdevice misc;
};

static irqreturn_t sim_irq(int irq, void *data)
{
	struct sim_bridge *b = data;
	unsigned int i;

	for (i = 0; i < SIM_NIRQ; i++)
		if (b->irq[i] == irq)
			break;
	if (i == SIM_NIRQ)
		return IRQ_NONE;
	spin_lock(&b->lock);
	if (b->enabled[i]) {
		disable_irq_nosync(irq);	/* one-shot until the runtime re-enables */
		b->enabled[i] = false;
	}
	b->pending |= BIT(i);
	spin_unlock(&b->lock);
	wake_up_interruptible(&b->wait);
	return IRQ_HANDLED;
}

static void sim_irq_set(struct sim_bridge *b, unsigned int i, bool on)
{
	unsigned long flags;

	spin_lock_irqsave(&b->lock, flags);
	if (on && !b->enabled[i]) {
		b->enabled[i] = true;
		enable_irq(b->irq[i]);
	} else if (!on && b->enabled[i]) {
		b->enabled[i] = false;
		disable_irq_nosync(b->irq[i]);
	}
	spin_unlock_irqrestore(&b->lock, flags);
}

static struct sim_bridge *to_bridge(struct file *file)
{
	return container_of(file->private_data, struct sim_bridge, misc);
}

static int sim_open(struct inode *inode, struct file *file)
{
	struct sim_bridge *b = to_bridge(file);

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (atomic_cmpxchg(&b->open, 0, 1))
		return -EBUSY;
	b->pending = 0;
	return 0;
}

static int sim_release(struct inode *inode, struct file *file)
{
	struct sim_bridge *b = to_bridge(file);
	unsigned int i;

	for (i = 0; i < SIM_NIRQ; i++)
		sim_irq_set(b, i, false);
	atomic_set(&b->open, 0);
	return 0;
}

static ssize_t sim_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	struct sim_bridge *b = to_bridge(file);
	unsigned long flags;
	u32 pending;
	int ret;

	if (count < 4)
		return -EINVAL;
	for (;;) {
		spin_lock_irqsave(&b->lock, flags);
		pending = b->pending;
		b->pending = 0;
		spin_unlock_irqrestore(&b->lock, flags);
		if (pending || (file->f_flags & O_NONBLOCK))
			break;
		ret = wait_event_interruptible(b->wait, READ_ONCE(b->pending));
		if (ret)
			return ret;
	}
	if (!pending && (file->f_flags & O_NONBLOCK))
		return -EAGAIN;
	return copy_to_user(buf, &pending, 4) ? -EFAULT : 4;
}

static __poll_t sim_poll(struct file *file, poll_table *wait)
{
	struct sim_bridge *b = to_bridge(file);

	poll_wait(file, &b->wait, wait);
	return READ_ONCE(b->pending) ? EPOLLIN | EPOLLRDNORM : 0;
}

static bool sim_pmic_write_ok(u32 dev, u32 reg)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sim_pmic_writes); i++)
		if (sim_pmic_writes[i].dev == dev && sim_pmic_writes[i].reg == reg)
			return true;
	return false;
}

static long sim_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sim_bridge *b = to_bridge(file);
	void __user *up = (void __user *)arg;
	int ret;

	switch (cmd) {
	case ISI_SIM_REG: {
		struct isi_sim_reg r;

		if (copy_from_user(&r, up, sizeof(r)))
			return -EFAULT;
		/* the firmware's own access sizes (upstream forwards its thumb ldr/str
		 * as they are); 0x20 is the byte-wide data FIFO: a 32-bit read of it
		 * bus-faults (measured with regpeek), so byte access only there */
		if (r.offset >= SIM_REGS || !(r.size == 1 || r.size == 2 || r.size == 4) ||
		    (r.offset & (r.size - 1)) || (r.offset >= 0x20 && r.offset < 0x24 && r.size != 1))
			return -EINVAL;
		if (r.write) {
			if (r.size == 4)
				writel(r.value, b->regs + r.offset);
			else if (r.size == 2)
				writew(r.value, b->regs + r.offset);
			else
				writeb(r.value, b->regs + r.offset);
		} else {
			r.value = r.size == 4 ? readl(b->regs + r.offset) :
				  r.size == 2 ? readw(b->regs + r.offset) : readb(b->regs + r.offset);
		}
		return copy_to_user(up, &r, sizeof(r)) ? -EFAULT : 0;
	}
	case ISI_SIM_PMIC: {
		struct isi_sim_pmic p;
		u16 old;

		if (copy_from_user(&p, up, sizeof(p)))
			return -EFAULT;
		if (p.device < 1 || p.device > 2 || p.reg >= 0x20 || p.value > 0xffff)
			return -EINVAL;
		if (p.write && !sim_pmic_write_ok(p.device, p.reg)) {
			dev_warn_ratelimited(b->misc.this_device, "PMIC write %u.%02x = %04x refused\n",
					     p.device, p.reg, p.value);
			return -EPERM;
		}
		ret = rapuyama_pmic_update(p.device, p.reg, p.write ? 0xffff : 0, p.value, &old);
		if (ret)
			return ret;
		if (!p.write)
			p.value = old;
		return copy_to_user(up, &p, sizeof(p)) ? -EFAULT : 0;
	}
	case ISI_SIM_IRQ: {
		struct isi_sim_irq q;

		if (copy_from_user(&q, up, sizeof(q)))
			return -EFAULT;
		if (q.number < SIM_IRQ_FIRST || q.number >= SIM_IRQ_FIRST + SIM_NIRQ)
			return -EINVAL;
		sim_irq_set(b, q.number - SIM_IRQ_FIRST, q.enable);
		return 0;
	}
	case ISI_SIM_ADC: {
		struct isi_sim_adc a;
		u16 raw;

		if (copy_from_user(&a, up, sizeof(a)))
			return -EFAULT;
		if (a.channel >= 19)
			return -EINVAL;
		ret = rapuyama_pmic_adc(a.channel, &raw);
		if (ret)
			return ret;
		a.value = raw;
		return copy_to_user(up, &a, sizeof(a)) ? -EFAULT : 0;
	}
	}
	return -ENOTTY;
}

static const struct file_operations sim_fops = {
	.owner = THIS_MODULE,
	.open = sim_open,
	.release = sim_release,
	.read = sim_read,
	.poll = sim_poll,
	.unlocked_ioctl = sim_ioctl,
};

static int sim_probe(struct platform_device *pdev)
{
	struct sim_bridge *b;
	struct resource *res;
	unsigned int i;
	int ret;

	b = devm_kzalloc(&pdev->dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != 0x20038300 || resource_size(res) != SIM_REGS)
		return -EINVAL;
	b->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(b->regs))
		return PTR_ERR(b->regs);
	spin_lock_init(&b->lock);
	init_waitqueue_head(&b->wait);
	for (i = 0; i < SIM_NIRQ; i++) {
		b->irq[i] = platform_get_irq(pdev, i);
		if (b->irq[i] < 0)
			return b->irq[i];
		ret = devm_request_irq(&pdev->dev, b->irq[i], sim_irq, IRQF_NO_AUTOEN,
				       "rapuyama-sim", b);
		if (ret)
			return ret;
	}
	b->misc.minor = MISC_DYNAMIC_MINOR;
	b->misc.name = "rapuyama-sim";
	b->misc.fops = &sim_fops;
	b->misc.parent = &pdev->dev;
	ret = misc_register(&b->misc);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, b);
	dev_info(&pdev->dev, "SIM bridge: regs %pa, irqs %d..%d, /dev/rapuyama-sim\n",
		 &res->start, b->irq[0], b->irq[SIM_NIRQ - 1]);
	return 0;
}

static void sim_remove(struct platform_device *pdev)
{
	struct sim_bridge *b = platform_get_drvdata(pdev);

	misc_deregister(&b->misc);
}

static const struct of_device_id sim_of_match[] = {
	{ .compatible = "nokia,e5-sim-bridge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sim_of_match);

static struct platform_driver sim_driver = {
	.probe = sim_probe,
	.remove = sim_remove,
	.driver = { .name = "rapuyama-sim", .of_match_table = sim_of_match },
};
module_platform_driver(sim_driver);
MODULE_DESCRIPTION("Nokia E5 SIM bridge for the userspace ISASW runtime");
MODULE_LICENSE("GPL");
