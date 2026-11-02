// SPDX-License-Identifier: GPL-2.0-only
/* Private, one-shot factory identity handoff. Never log the value. */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/rapuyama_identity.h>
#include <asm/cacheflush.h>

static unsigned char factory_identity[16];
static bool identity_present;
static DEFINE_MUTEX(identity_lock);

static ssize_t identity_read(struct file *file, char __user *buffer,
			     size_t size, loff_t *offset)
{
	ssize_t result = 0;
	if (size < sizeof(factory_identity)) return -EINVAL;
	if (mutex_lock_interruptible(&identity_lock)) return -ERESTARTSYS;
	if (!identity_present || *offset) goto out;
	if (copy_to_user(buffer, factory_identity, sizeof(factory_identity))) {
		result = -EFAULT;
		goto out;
	}
	memzero_explicit(factory_identity, sizeof(factory_identity));
	identity_present = false;
	*offset += 16;
	result = 16;
out:
	mutex_unlock(&identity_lock);
	return result;
}
static const struct file_operations identity_fops = {
	.owner = THIS_MODULE,
	.open = nonseekable_open,
	.read = identity_read,
};
static struct miscdevice identity_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rapuyama-identity",
	.fops = &identity_fops,
	.mode = 0400,
};
static int __init rapuyama_identity_init(void)
{
	unsigned char packet[E6_ID_SIZE];
	u32 *handoff;
	void *slot;
	int result;
	if (!of_machine_is_compatible("nokia,rapuyama")) return 0;
	handoff = memremap(0x1f0f1000, PAGE_SIZE, MEMREMAP_WB);
	if (!handoff) return 0;
	slot = (unsigned char *)handoff + E6_ID_OFFSET;
	memcpy(packet, slot, sizeof(packet));
	memzero_explicit(slot, E6_ID_SIZE);
	__cpuc_flush_dcache_area(slot, E6_ID_SIZE);
	wmb();
	if (handoff[0] == 0x48415349)
		identity_present = e6_id_unpack(packet, factory_identity);
	memzero_explicit(packet, sizeof(packet));
	memunmap(handoff);
	if (!identity_present) return 0;
	factory_identity[15] = '\n';
	result = misc_register(&identity_device);
	if (result) {
		memzero_explicit(factory_identity, sizeof(factory_identity));
		identity_present = false;
	}
	return result;
}
late_initcall(rapuyama_identity_init);
