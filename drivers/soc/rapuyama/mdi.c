// SPDX-License-Identifier: GPL-2.0-only
#include <linux/capability.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/rapuyama_mdi.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>

/*
 * Where the preserved firmware's RAM and the DSP buffer window are, per phone.
 * ap = the ISASW data (runtime private copy at ap_virt), dsp = the DSP shared
 * buffers (runtime copy at dsp_virt). Descriptors carry physical - 0x10000000.
 * The PA_SWD bridge borrows a stopped task's stack inside the ISASW data (see
 * secure_wdt.c); copied buffers never overlap it.
 */
struct mdi_layout {
	const char *compat;
	u32 ap_phys, ap_virt, ap_size;
	u32 dsp_phys, dsp_virt, dsp_size;
	u32 swd_stack, swd_stack_size;
};
static const struct mdi_layout mdi_layouts[] = {
	{ "nokia,e6", 0x1f8b0000, 0xc6800000, 0x44d000,
	  0x10140000, 0x90c00000, 0xbf000, 0xf8000, 0x6000 },
	/* E5 fw 101.003: ISASW data pa 0x1fd50000 (va c6800000, 2.5 MB); DSP
	 * buffers pa 0x10140000 = Symbian va cb500000 (512 KB, the rest of that
	 * MB is other memory); PA_SWD stack sp c68d7e6c */
	{ "nokia,e5", 0x1fd50000, 0xc6800000, 0x280000,
	  0x10140000, 0xcb500000, 0x80000, 0xd2000, 0x6000 },
};
static const struct mdi_layout *L;
#define MDI_AP_PHYS		(L->ap_phys)
#define MDI_AP_VIRT		(L->ap_virt)
#define MDI_AP_SIZE		(L->ap_size)
#define MDI_DSP_PHYS		(L->dsp_phys)
#define MDI_DSP_VIRT		(L->dsp_virt)
#define MDI_DSP_SIZE		(L->dsp_size)
#define MDI_DSP_PHYS_BASE	0x10000000
#define MDI_AP_SWD_STACK	(L->swd_stack)
#define MDI_AP_SWD_STACK_SIZE	(L->swd_stack_size)
#define MDI_AP			0
#define MDI_DSP			0x110
#define MDI_SLOTS		16
#define MDI_SET			4
#define MDI_CLEAR		8
#define MDI_SLOT(index)		(4 + ((index) & 15) * 16)

struct rapuyama_mdi {
	struct miscdevice misc;
	struct kref refs;
	struct device *dev;
	struct resource ring_res, ctrl_res;
	void __iomem *ring, *ctrl, *dsp_memory;
	void *ap_memory;
	struct mutex io;
	wait_queue_head_t wait;
	atomic_t sequence, rx_interrupts, tx_interrupts, event_interrupts;
	bool attached, dying;
	int irq_rx, irq_tx, irq_event;
};

struct mdi_file {
	struct rapuyama_mdi *mdi;
	unsigned int events;
	struct mm_struct *mm;
	bool buffers;
	bool dsp_direct;	/* buffers mode 2: the runtime maps the DSP window (mmap), no copies */
};

static void mdi_free(struct kref *refs)
{
	struct rapuyama_mdi *mdi = container_of(refs, struct rapuyama_mdi, refs);

	if (mdi->ctrl)
		iounmap(mdi->ctrl);
	if (mdi->ring)
		iounmap(mdi->ring);
	release_mem_region(mdi->ctrl_res.start, resource_size(&mdi->ctrl_res));
	release_mem_region(mdi->ring_res.start, resource_size(&mdi->ring_res));
	if (mdi->ap_memory)
		memunmap(mdi->ap_memory);
	if (mdi->dsp_memory)
		iounmap(mdi->dsp_memory);
	put_device(mdi->dev);
	kfree(mdi);
}

static int mdi_state(struct rapuyama_mdi *mdi, u32 *ap, u32 *dsp)
{
	u32 before = readl(mdi->ring + MDI_DSP), after;
	int tries;

	for (tries = 0; tries < 64; tries++) {
		after = readl(mdi->ring + MDI_DSP);
		if (before == after) {
			*dsp = after;
			*ap = readl(mdi->ring + MDI_AP);
			if ((*ap | *dsp) & 0xffe0ffe0U)
				return -EPROTO;
			return 0;
		}
		before = after;
	}
	return -EAGAIN;
}

static int mdi_used(u32 producer, u32 consumer)
{
	u32 used = (producer - consumer) & 31;

	return used <= MDI_SLOTS ? used : -EPROTO;
}

static irqreturn_t mdi_rx_irq(int irq, void *data)
{
	struct rapuyama_mdi *mdi = data;

	writel(2, mdi->ctrl + MDI_CLEAR);
	readl(mdi->ctrl);
	atomic_inc(&mdi->rx_interrupts);
	atomic_inc(&mdi->sequence);
	wake_up_interruptible(&mdi->wait);
	return IRQ_HANDLED;
}

static irqreturn_t mdi_tx_irq(int irq, void *data)
{
	struct rapuyama_mdi *mdi = data;

	writel(8, mdi->ctrl + MDI_CLEAR);
	readl(mdi->ctrl);
	atomic_inc(&mdi->tx_interrupts);
	atomic_inc(&mdi->sequence);
	wake_up_interruptible(&mdi->wait);
	return IRQ_HANDLED;
}

static irqreturn_t mdi_event_irq(int irq, void *data)
{
	struct rapuyama_mdi *mdi = data;

	writel(0x20, mdi->ctrl + MDI_CLEAR);
	readl(mdi->ctrl);
	atomic_inc(&mdi->event_interrupts);
	atomic_inc(&mdi->sequence);
	wake_up_interruptible(&mdi->wait);
	return IRQ_HANDLED;
}

static int mdi_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct rapuyama_mdi *mdi = container_of(misc, struct rapuyama_mdi, misc);
	struct mdi_file *context;
	int ret = 0;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	context = kzalloc(sizeof(*context), GFP_KERNEL);
	if (!context)
		return -ENOMEM;
	mutex_lock(&mdi->io);
	if (mdi->dying)
		ret = -ENODEV;
	else if (mdi->attached)
		ret = -EBUSY;
	else {
		mdi->attached = true;
		kref_get(&mdi->refs);
		context->mdi = mdi;
		context->events = atomic_read(&mdi->sequence);
		file->private_data = context;
	}
	mutex_unlock(&mdi->io);
	if (ret) {
		kfree(context);
		return ret;
	}
	return stream_open(inode, file);
}

static int mdi_release(struct inode *inode, struct file *file)
{
	struct mdi_file *context = file->private_data;
	struct rapuyama_mdi *mdi = context->mdi;

	mutex_lock(&mdi->io);
	mdi->attached = false;
	mutex_unlock(&mdi->io);
	if (context->mm)
		mmput(context->mm);
	kref_put(&mdi->refs, mdi_free);
	kfree(context);
	return 0;
}

static int mdi_tx_buffer(struct mdi_file *context, const __le32 descriptor[4])
{
	struct rapuyama_mdi *mdi = context->mdi;
	u64 physical;
	u32 offset, size;
	void *copy;
	int ret = 0;

	if (!context->buffers)
		return 0;
	if (context->mm != current->mm)
		return -EXDEV;
	if (le32_to_cpu(descriptor[2]) >> 24 != 6)
		return 0;
	physical = (u64)le32_to_cpu(descriptor[3]) + MDI_DSP_PHYS_BASE;
	size = swab16(le32_to_cpu(descriptor[1]) & 0xffff) + 6;
	if (size < 15 || (physical & 3) || physical < MDI_AP_PHYS ||
	    physical + size > MDI_AP_PHYS + MDI_AP_SIZE)
		return -EPROTO;
	offset = physical - MDI_AP_PHYS;
	if (offset + size > MDI_AP_SWD_STACK &&
	    offset < MDI_AP_SWD_STACK + MDI_AP_SWD_STACK_SIZE)
		return -EPROTO;
	copy = memdup_user((void __user *)(unsigned long)(MDI_AP_VIRT + offset),
			   size);
	if (IS_ERR(copy))
		return PTR_ERR(copy);
	if (get_unaligned_le32(copy) != le32_to_cpu(descriptor[0]) ||
	    get_unaligned_le32(copy + 4) != le32_to_cpu(descriptor[1]) ||
	    get_unaligned_le16(copy + 8) !=
		    (le32_to_cpu(descriptor[2]) & 0xffff)) {
		ret = -EPROTO;
		goto free;
	}
	memcpy(mdi->ap_memory + offset, copy, size);
	__cpuc_flush_dcache_area(mdi->ap_memory + offset, size);
	outer_clean_range(physical, physical + size);
free:
	kfree(copy);
	return ret;
}

static int mdi_rx_buffer(struct mdi_file *context, const __le32 descriptor[4])
{
	struct rapuyama_mdi *mdi = context->mdi;
	u64 physical;
	u32 offset, size;
	void *copy;
	int ret = 0;

	if (!context->buffers || context->dsp_direct)
		return 0;
	if (context->mm != current->mm)
		return -EXDEV;
	if (((le32_to_cpu(descriptor[2]) >> 24) & 0x1e) != 0x16)
		return 0;
	physical = (u64)le32_to_cpu(descriptor[3]) + MDI_DSP_PHYS_BASE;
	size = swab16(le32_to_cpu(descriptor[1]) & 0xffff) + 10;
	if (size < 14 || (physical & 3) || physical < MDI_DSP_PHYS ||
	    physical + size > MDI_DSP_PHYS + MDI_DSP_SIZE)
		return -EPROTO;
	offset = physical - MDI_DSP_PHYS;

	copy = kmalloc(size, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	memcpy_fromio(copy, mdi->dsp_memory + offset, size);
	if (get_unaligned_be32(copy) != le32_to_cpu(descriptor[0])) {
		ret = -EPROTO;
		goto free;
	}
	if (copy_to_user((void __user *)(unsigned long)(MDI_DSP_VIRT + offset),
			 copy, size))
		ret = -EFAULT;
free:
	kfree(copy);
	return ret;
}

static ssize_t mdi_read(struct file *file, char __user *buffer, size_t count,
			loff_t *offset)
{
	struct mdi_file *context = file->private_data;
	struct rapuyama_mdi *mdi = context->mdi;
	__le32 descriptor[4];
	u32 ap, dsp;
	int ret, seen, used, i;

	if (!count)
		return 0;
	if (count < sizeof(descriptor))
		return -EMSGSIZE;
	for (;;) {
		ret = mutex_lock_interruptible(&mdi->io);
		if (ret)
			return ret;
		if (mdi->dying) {
			ret = -ENODEV;
			goto unlock;
		}
		seen = atomic_read(&mdi->sequence);
		ret = mdi_state(mdi, &ap, &dsp);
		if (ret)
			goto unlock;
		used = mdi_used(dsp & 31, ap >> 16);
		if (used < 0) {
			ret = used;
			goto unlock;
		}
		if (used) {
			rmb();
			for (i = 0; i < 4; i++)
				descriptor[i] = cpu_to_le32(readl(
					mdi->ring + MDI_DSP + MDI_SLOT(ap >> 16) +
					i * 4));
			ret = mdi_rx_buffer(context, descriptor);
			if (ret)
				goto unlock;
			if (copy_to_user(buffer, descriptor, sizeof(descriptor))) {
				ret = -EFAULT;
				goto unlock;
			}
			ap = (ap & 0xffff) | ((((ap >> 16) + 1) & 31) << 16);
			wmb();
			writel(ap, mdi->ring + MDI_AP);
			readl(mdi->ring + MDI_AP);
			writel(4, mdi->ctrl + MDI_SET);
			ret = sizeof(descriptor);
			goto unlock;
		}
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto unlock;
		}
		mutex_unlock(&mdi->io);
		ret = wait_event_interruptible(mdi->wait,
			READ_ONCE(mdi->dying) ||
				atomic_read(&mdi->sequence) != seen);
		if (ret)
			return ret;
	}
unlock:
	mutex_unlock(&mdi->io);
	return ret;
}

static ssize_t mdi_write(struct file *file, const char __user *buffer,
			 size_t count, loff_t *offset)
{
	struct mdi_file *context = file->private_data;
	struct rapuyama_mdi *mdi = context->mdi;
	__le32 descriptor[4];
	u32 ap, dsp;
	int ret, seen, used, i;

	if (count != sizeof(descriptor))
		return -EMSGSIZE;
	if (copy_from_user(descriptor, buffer, sizeof(descriptor)))
		return -EFAULT;
	for (;;) {
		ret = mutex_lock_interruptible(&mdi->io);
		if (ret)
			return ret;
		if (mdi->dying) {
			ret = -ENODEV;
			goto unlock;
		}
		seen = atomic_read(&mdi->sequence);
		ret = mdi_state(mdi, &ap, &dsp);
		if (ret)
			goto unlock;
		used = mdi_used(ap & 31, dsp >> 16);
		if (used < 0) {
			ret = used;
			goto unlock;
		}
		if (used < MDI_SLOTS) {
			ret = mdi_tx_buffer(context, descriptor);
			if (ret)
				goto unlock;
			for (i = 0; i < 4; i++)
				writel(le32_to_cpu(descriptor[i]),
				       mdi->ring + MDI_AP + MDI_SLOT(ap & 31) +
					       i * 4);
			readl(mdi->ring + MDI_AP + MDI_SLOT(ap & 31) + 12);
			ap = (ap & 0xffff0000) | (((ap & 31) + 1) & 31);
			wmb();
			writel(ap, mdi->ring + MDI_AP);
			readl(mdi->ring + MDI_AP);
			writel(8, mdi->ctrl + MDI_CLEAR);
			writel(1, mdi->ctrl + MDI_SET);
			ret = sizeof(descriptor);
			goto unlock;
		}
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto unlock;
		}
		mutex_unlock(&mdi->io);
		ret = wait_event_interruptible(mdi->wait,
			READ_ONCE(mdi->dying) ||
				atomic_read(&mdi->sequence) != seen);
		if (ret)
			return ret;
	}
unlock:
	mutex_unlock(&mdi->io);
	return ret;
}

static __poll_t mdi_poll(struct file *file, poll_table *wait)
{
	struct mdi_file *context = file->private_data;
	struct rapuyama_mdi *mdi = context->mdi;
	__poll_t mask = 0;
	u32 ap, dsp;
	int rx, tx, ret;

	poll_wait(file, &mdi->wait, wait);
	mutex_lock(&mdi->io);
	if (mdi->dying) {
		mask = EPOLLERR | EPOLLHUP;
		goto unlock;
	}
	ret = mdi_state(mdi, &ap, &dsp);
	if (ret) {
		mask = EPOLLERR;
		goto unlock;
	}
	rx = mdi_used(dsp & 31, ap >> 16);
	tx = mdi_used(ap & 31, dsp >> 16);
	if (rx < 0 || tx < 0) {
		mask = EPOLLERR;
		goto unlock;
	}
	if (atomic_read(&mdi->sequence) != READ_ONCE(context->events))
		mask |= EPOLLPRI;
	if (rx)
		mask |= EPOLLIN | EPOLLRDNORM;
	if (tx < MDI_SLOTS)
		mask |= EPOLLOUT | EPOLLWRNORM;
unlock:
	mutex_unlock(&mdi->io);
	return mask;
}

static int mdi_dsp_snapshot(struct rapuyama_mdi *mdi)
{
	void *page = (void *)__get_free_page(GFP_KERNEL);
	u32 offset, size;
	int ret = 0;

	if (!page)
		return -ENOMEM;
	for (offset = 0; offset < MDI_DSP_SIZE; offset += size) {
		size = min_t(u32, PAGE_SIZE, MDI_DSP_SIZE - offset);
		memcpy_fromio(page, mdi->dsp_memory + offset, size);
		if (copy_to_user((void __user *)(unsigned long)(MDI_DSP_VIRT +
							       offset),
				 page, size)) {
			ret = -EFAULT;
			break;
		}
	}
	free_page((unsigned long)page);
	return ret;
}

static long mdi_ioctl(struct file *file, unsigned int command,
		      unsigned long argument)
{
	struct mdi_file *context = file->private_data;
	struct rapuyama_mdi *mdi = context->mdi;
	struct rapuyama_mdi_status status = { .version = 2 };
	unsigned int sequence;
	int ret;
	u32 buffers;

	if (command == RAPUYAMA_MDI_BUFFERS) {
		if (copy_from_user(&buffers, (void __user *)argument,
				   sizeof(buffers)))
			return -EFAULT;
		if (buffers > 2)
			return -EINVAL;
		ret = mutex_lock_interruptible(&mdi->io);
		if (ret)
			return ret;
		if (mdi->dying) {
			ret = -ENODEV;
			goto unlock;
		}
		if (!buffers) {
			context->buffers = false;
			context->dsp_direct = false;
			if (context->mm) {
				mmput(context->mm);
				context->mm = NULL;
			}
			ret = 0;
			goto unlock;
		}
		if (!access_ok((void __user *)(unsigned long)MDI_AP_VIRT,
			       MDI_AP_SIZE) ||
		    !access_ok((void __user *)(unsigned long)MDI_DSP_VIRT,
			       MDI_DSP_SIZE)) {
			ret = -EINVAL;
			goto unlock;
		}
		if (context->mm && context->mm != current->mm) {
			ret = -EXDEV;
			goto unlock;
		}
		if (!mdi->ap_memory)
			mdi->ap_memory = memremap(MDI_AP_PHYS, MDI_AP_SIZE,
						  MEMREMAP_WB);
		if (!mdi->dsp_memory)
			mdi->dsp_memory = ioremap(MDI_DSP_PHYS, MDI_DSP_SIZE);
		if (!mdi->ap_memory || !mdi->dsp_memory) {
			ret = -ENOMEM;
			goto unlock;
		}
		if (!context->mm)
			context->mm = get_task_mm(current);
		if (!context->mm) {
			ret = -EINVAL;
			goto unlock;
		}
		context->dsp_direct = buffers == 2;
		if (!context->buffers && !context->dsp_direct) {
			ret = mdi_dsp_snapshot(mdi);
			if (ret)
				goto unlock;
		}
		context->buffers = true;
		ret = 0;
		goto unlock;
	}
	if (command != RAPUYAMA_MDI_STATUS)
		return -ENOTTY;
	ret = mutex_lock_interruptible(&mdi->io);
	if (ret)
		return ret;
	if (mdi->dying) {
		ret = -ENODEV;
		goto unlock;
	}
	sequence = atomic_read(&mdi->sequence);
	ret = mdi_state(mdi, &status.ap, &status.dsp);
	if (ret)
		goto unlock;
	status.rx_interrupts = atomic_read(&mdi->rx_interrupts);
	status.tx_interrupts = atomic_read(&mdi->tx_interrupts);
	status.event_interrupts = atomic_read(&mdi->event_interrupts);
	if (copy_to_user((void __user *)argument, &status, sizeof(status))) {
		ret = -EFAULT;
		goto unlock;
	}
	WRITE_ONCE(context->events, sequence);
unlock:
	mutex_unlock(&mdi->io);
	return ret;
}

/*
 * The DSP buffer window itself (MDI_DSP_PHYS, outside Linux RAM), write-combine
 * (normal uncached: the firmware may access it unaligned). With buffers mode 2
 * the runtime uses it at MDI_DSP_VIRT instead of the copy: the DSP also reads
 * and writes buffers no descriptor names (e.g. the DCH transport-block ring).
 */
static int mdi_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;

	if (vma->vm_pgoff || size != MDI_DSP_SIZE)
		return -EINVAL;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start, MDI_DSP_PHYS >> PAGE_SHIFT,
			       size, vma->vm_page_prot);
}

static const struct file_operations mdi_fops = {
	.owner = THIS_MODULE,
	.open = mdi_open,
	.release = mdi_release,
	.read = mdi_read,
	.write = mdi_write,
	.poll = mdi_poll,
	.unlocked_ioctl = mdi_ioctl,
	.mmap = mdi_mmap,
};

static int mdi_probe(struct platform_device *pdev)
{
	struct resource *ring = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *ctrl = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	struct rapuyama_mdi *mdi;
	u32 ap, dsp;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mdi_layouts); i++)
		if (of_machine_is_compatible(mdi_layouts[i].compat))
			L = &mdi_layouts[i];
	if (!L || !ring || !ctrl ||
	    ring->start != 0x30003ce0 || resource_size(ring) != 0x220 ||
	    ctrl->start != 0x20040280 || resource_size(ctrl) != 12)
		return -EINVAL;
	mdi = kzalloc(sizeof(*mdi), GFP_KERNEL);
	if (!mdi)
		return -ENOMEM;
	mdi->dev = get_device(&pdev->dev);
	mdi->ring_res = *ring;
	mdi->ctrl_res = *ctrl;
	kref_init(&mdi->refs);
	mutex_init(&mdi->io);
	init_waitqueue_head(&mdi->wait);
	atomic_set(&mdi->sequence, 0);
	atomic_set(&mdi->rx_interrupts, 0);
	atomic_set(&mdi->tx_interrupts, 0);
	atomic_set(&mdi->event_interrupts, 0);
	if (!request_mem_region(ring->start, resource_size(ring),
				dev_name(&pdev->dev))) {
		ret = -EBUSY;
		goto free;
	}
	if (!request_mem_region(ctrl->start, resource_size(ctrl),
				dev_name(&pdev->dev))) {
		ret = -EBUSY;
		goto release_ring;
	}
	mdi->ring = ioremap(ring->start, resource_size(ring));
	mdi->ctrl = ioremap(ctrl->start, resource_size(ctrl));
	if (!mdi->ring || !mdi->ctrl) {
		ret = -ENOMEM;
		goto put;
	}
	ret = mdi_state(mdi, &ap, &dsp);
	if (ret)
		goto put;
	if (mdi_used(ap & 31, dsp >> 16) < 0 ||
	    mdi_used(dsp & 31, ap >> 16) < 0) {
		ret = -EPROTO;
		goto put;
	}
	mdi->irq_rx = platform_get_irq_byname(pdev, "rx");
	if (mdi->irq_rx < 0) {
		ret = mdi->irq_rx;
		goto put;
	}
	mdi->irq_tx = platform_get_irq_byname(pdev, "tx");
	if (mdi->irq_tx < 0) {
		ret = mdi->irq_tx;
		goto put;
	}
	ret = request_irq(mdi->irq_rx, mdi_rx_irq, 0, "rapuyama-mdi-rx", mdi);
	if (ret)
		goto put;
	ret = request_irq(mdi->irq_tx, mdi_tx_irq, 0, "rapuyama-mdi-tx", mdi);
	if (ret)
		goto free_rx;
	mdi->irq_event = platform_get_irq_byname_optional(pdev, "event");
	if (mdi->irq_event > 0) {
		ret = request_irq(mdi->irq_event, mdi_event_irq, 0,
				  "rapuyama-mdi-event", mdi);
		if (ret)
			goto free_tx;
	}
	mdi->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "rapuyama-mdi",
		.fops = &mdi_fops,
		.parent = &pdev->dev,
		.mode = 0600,
	};
	ret = misc_register(&mdi->misc);
	if (ret)
		goto free_event;
	platform_set_drvdata(pdev, mdi);
	dev_info(&pdev->dev, "adopted MDI rings ap=%08x dsp=%08x\n", ap, dsp);
	return 0;
free_event:
	if (mdi->irq_event > 0)
		free_irq(mdi->irq_event, mdi);
free_tx:
	free_irq(mdi->irq_tx, mdi);
free_rx:
	free_irq(mdi->irq_rx, mdi);
put:
	kref_put(&mdi->refs, mdi_free);
	return ret;
release_ring:
	release_mem_region(ring->start, resource_size(ring));
free:
	put_device(mdi->dev);
	kfree(mdi);
	return ret;
}

static void mdi_remove(struct platform_device *pdev)
{
	struct rapuyama_mdi *mdi = platform_get_drvdata(pdev);

	mutex_lock(&mdi->io);
	mdi->dying = true;
	mutex_unlock(&mdi->io);
	wake_up_interruptible(&mdi->wait);
	misc_deregister(&mdi->misc);
	if (mdi->irq_event > 0)
		free_irq(mdi->irq_event, mdi);
	free_irq(mdi->irq_tx, mdi);
	free_irq(mdi->irq_rx, mdi);
	kref_put(&mdi->refs, mdi_free);
}

static const struct of_device_id mdi_match[] = {
	{ .compatible = "nokia,e6-mdi" },
	{ .compatible = "nokia,e5-mdi" },
	{ }
};
MODULE_DEVICE_TABLE(of, mdi_match);

static struct platform_driver mdi_driver = {
	.probe = mdi_probe,
	.remove = mdi_remove,
	.driver = {
		.name = "rapuyama-mdi",
		.of_match_table = mdi_match,
	},
};
module_platform_driver(mdi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nokia E6 MCU to DSP MDI descriptor transport");
