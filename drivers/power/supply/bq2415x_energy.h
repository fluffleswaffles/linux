/* SPDX-License-Identifier: GPL-2.0-or-later */
/* e6 em owns both chargers */
#ifdef CONFIG_ARCH_RAPUYAMA
#include <linux/capability.h>
#include <linux/compat.h>
#include <linux/kref.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/soc/rapuyama-gpio.h>
#include <linux/soc/rapuyama-pmic.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <uapi/linux/rapuyama_energy.h>

#define ENERGY_LEASE (5 * HZ)

struct bq2415x_energy {
	struct mutex lock;
	struct kref ref;
	struct miscdevice misc;
	struct delayed_work guard;
	struct bq2415x_device *bq;
	struct i2c_client *dynamic;
	struct gpio_desc *stat;
	int stat_irq;
	atomic_t irq_enabled, pending;
	unsigned long deadline;
	u32 inhibit, budget;
	bool opened, active, revoked, stopping;
};

static void energy_irq_control(struct bq2415x_energy *e, bool enable)
{
	if (enable) {
		synchronize_irq(e->stat_irq);
		if (atomic_read(&e->pending))
			return;
		if (!atomic_xchg(&e->irq_enabled, 1))
			enable_irq(e->stat_irq);
	} else if (atomic_xchg(&e->irq_enabled, 0)) {
		disable_irq(e->stat_irq);
	} else {
		synchronize_irq(e->stat_irq);
	}
}

static irqreturn_t energy_irq(int irq, void *data)
{
	struct bq2415x_energy *e = data;

	if (atomic_xchg(&e->irq_enabled, 0))
		disable_irq_nosync(irq);
	if (READ_ONCE(e->active) && !READ_ONCE(e->revoked) && !READ_ONCE(e->stopping)) {
		atomic_set(&e->pending, 1);
		mod_delayed_work(system_wq, &e->guard, 0);
	}
	return IRQ_HANDLED;
}

static int energy_transfer(struct bq2415x_energy *e, unsigned int chip,
			   u8 reg, u8 *value, bool write)
{
	struct i2c_client *client = chip ? e->dynamic : to_i2c_client(e->bq->dev);
	u8 bytes[2] = {reg, *value};
	struct i2c_msg msg[2] = {
		{ .addr = client->addr, .len = write ? 2 : 1, .buf = bytes },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = value },
	};
	int count = write ? 1 : 2, ret;

	mutex_lock(&bq2415x_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, count);
	mutex_unlock(&bq2415x_i2c_mutex);
	return ret < 0 ? ret : ret == count ? 0 : -EIO;
}

static int energy_read(struct bq2415x_energy *e, unsigned int chip, u8 reg, u8 *value)
{
	*value = 0;
	return energy_transfer(e, chip, reg, value, false);
}

static int energy_write(struct bq2415x_energy *e, unsigned int chip, u8 reg, u8 value)
{
	return energy_transfer(e, chip, reg, &value, true);
}

static int energy_stop(struct bq2415x_energy *e, unsigned int chip)
{
	u8 value;
	int ret;
	ret = energy_read(e, chip, 1, &value);
	if (ret)
		value = 0;
	ret = energy_write(e, chip, 1, (value & ~1) | 6);
	if (ret)
		return ret;
	ret = energy_read(e, chip, 1, &value);
	return ret ? ret : (value & 7) == 6 ? 0 : -EIO;
}

static int energy_stop_all(struct bq2415x_energy *e)
{
	int usb = energy_stop(e, 0);
	int dynamic = energy_stop(e, 1);
#if IS_REACHABLE(CONFIG_RAPUYAMA_WATCHDOG)
	int pmic = rapuyama_pmic_em_stop();
#else
	int pmic = -ENODEV;
#endif

	return usb ? usb : dynamic ? dynamic : pmic;
}

static void energy_revoke(struct bq2415x_energy *e)
{
	WRITE_ONCE(e->revoked, true);
	energy_irq_control(e, false);
	atomic_set(&e->pending, 0);
	if (energy_stop_all(e))
		dev_err(e->bq->dev, "EM lease revoked; charger disable could not be verified\n");
	power_supply_changed(e->bq->charger);
}

static int energy_check(struct bq2415x_energy *e, unsigned int chip)
{
	union power_supply_propval temp, voltage, budget;
	u8 source;
	int ret;

	ret = power_supply_get_property(e->bq->battery, POWER_SUPPLY_PROP_TEMP, &temp);
	if (ret)
		return ret;
	ret = power_supply_get_property(e->bq->battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &voltage);
	if (ret)
		return ret;
	if (temp.intval < (e->inhibit & RAPUYAMA_ENERGY_THERMAL ? 70 : 50) ||
	    temp.intval > (e->inhibit & RAPUYAMA_ENERGY_THERMAL ? 380 : 400) ||
	    voltage.intval < 2500000 || voltage.intval > 4250000) {
		e->inhibit |= RAPUYAMA_ENERGY_THERMAL;
		return -EAGAIN;
	}
	e->inhibit &= ~RAPUYAMA_ENERGY_THERMAL;
	if (!chip) {
		ret = power_supply_get_property(e->bq->source, POWER_SUPPLY_PROP_CURRENT_MAX, &budget);
		if (ret)
			return ret;
		e->budget = max(budget.intval, 0);
		if (e->budget < 500000) {
			e->inhibit |= RAPUYAMA_ENERGY_USB_BLOCKED;
			return -EAGAIN;
		}
		e->inhibit &= ~RAPUYAMA_ENERGY_USB_BLOCKED;
	} else {
		ret = energy_read(e, 0, 5, &source);
		if (ret)
			return ret;
		if (!(source & 8)) {
			e->inhibit |= RAPUYAMA_ENERGY_DYNAMIC_BLOCKED;
			return -EAGAIN;
		}
		e->inhibit &= ~RAPUYAMA_ENERGY_DYNAMIC_BLOCKED;
	}
	return 0;
}

static int energy_limits(struct bq2415x_energy *e, unsigned int chip)
{
	u8 control, voltage, current_reg;
	int ret;

	ret = energy_read(e, chip, 1, &control);
	if (!ret)
		ret = energy_read(e, chip, 2, &voltage);
	if (!ret)
		ret = energy_read(e, chip, 4, &current_reg);
	if (ret)
		return ret;
	if ((control & 1) || (voltage & 1) || (voltage >> 2) > 35 ||
	    (current_reg & 128) || ((current_reg >> (chip ? 3 : 4)) & (chip ? 15 : 7)) > 4 ||
	    (!chip && (control >> 6) > 1))
		return -ERANGE;
	return 0;
}

static void energy_guard(struct work_struct *work)
{
	struct bq2415x_energy *e = container_of(work, struct bq2415x_energy, guard.work);
	unsigned int chip;
	u8 control, status, other;
	int ret;

	mutex_lock(&e->lock);
	if (!e->bq || e->stopping || !e->active || e->revoked)
		goto out;
	if (time_after_eq(jiffies, e->deadline)) {
		energy_revoke(e);
		goto out;
	}
	for (chip = 0; chip < 2; chip++) {
		ret = energy_read(e, chip, 1, &control);
		if (!ret)
			ret = energy_read(e, chip, 0, &status);
		if (ret || (control & 1) || (status & 7) == 1 || (status & 7) == 4 || (status & 7) == 5 ||
		    (status & 7) == 6) {
			energy_revoke(e);
			goto out;
		}
		if (control & 6)
			continue;
		ret = energy_read(e, !chip, 1, &other);
		if (ret || !(other & 6)) {
			energy_revoke(e);
			goto out;
		}
		ret = energy_check(e, chip);
		if (!ret)
			ret = energy_limits(e, chip);
		if (ret == -EAGAIN || (!ret && (status & 7))) {
			ret = energy_stop(e, chip);
			if (!ret) {
				power_supply_changed(e->bq->charger);
				continue;
			}
		}
		if (ret) {
			energy_revoke(e);
			goto out;
		}
	}
	mod_delayed_work(system_wq, &e->guard, HZ);
out:
	mutex_unlock(&e->lock);
}

static void bq2415x_energy_notify(struct bq2415x_device *bq)
{
	struct bq2415x_energy *e = bq->energy;

	if (!e)
		return;
	mutex_lock(&e->lock);
	if (e->bq && !e->stopping && e->active && !e->revoked)
		mod_delayed_work(system_wq, &e->guard, 0);
	mutex_unlock(&e->lock);
}

static int energy_io(struct bq2415x_energy *e, struct rapuyama_energy_io *q)
{
	unsigned int chip;
	u8 value, other, mask;
	int ret;

	if ((q->address != 0x6a && q->address != 0x6b) || q->reg > 6 ||
	    q->value > 255 || q->write > 1)
		return -EINVAL;
	chip = q->address == 0x6a;
	if (!e->active || e->revoked)
		return -EOWNERDEAD;
	if (time_after_eq(jiffies, e->deadline)) {
		energy_revoke(e);
		return -EOWNERDEAD;
	}
	if (!q->write) {
		ret = energy_read(e, chip, q->reg, &value);
		if (!ret)
			q->value = value;
		goto done;
	}
	if (q->reg == 3 || q->reg == 6 ||
	    (q->reg == 1 && (q->value & 1)) ||
	    (q->reg == 2 && ((q->value & 1) || (q->value >> 2) > 35)) ||
	    (q->reg == 4 && ((q->value & 128) ||
		((q->value >> (chip ? 3 : 4)) & (chip ? 15 : 7)) > 4)))
		return -EPERM;
	if (q->reg == 5) {
		ret = energy_read(e, chip, 5, &value);
		if (ret)
			goto done;
		if ((value ^ q->value) & ~0x27)
			return -EPERM;
	}
	if (q->reg != 1) {
		ret = energy_read(e, chip, 1, &value);
		if (ret)
			goto done;
		if (!(value & 6)) {
			ret = energy_check(e, chip);
			if (ret == -EAGAIN) {
				ret = energy_stop(e, chip);
				if (!ret) {
					power_supply_changed(e->bq->charger);
					return -EAGAIN;
				}
			}
			if (ret)
				goto done;
		}
	}
	if (q->reg == 1 && !(q->value & 6)) {
		ret = energy_check(e, chip);
		if (ret == -EAGAIN) {
			ret = energy_stop(e, chip);
			if (!ret) {
				power_supply_changed(e->bq->charger);
				return -EAGAIN;
			}
		}
		if (!ret)
			ret = energy_limits(e, chip);
		if (!ret)
			ret = energy_read(e, !chip, 1, &other);
		if (ret)
			goto done;
		if (!(other & 6))
			return -EAGAIN;
	}
	if (q->reg == 1 && !chip && (q->value >> 6) > 1)
		return -EPERM;
	ret = energy_write(e, chip, q->reg, q->value);
	if (!ret && q->reg != 0) {
		ret = energy_read(e, chip, q->reg, &value);
		mask = q->reg == 5 ? 0x27 : q->reg == 4 && !chip ? 0xf7 : 0xff;
		if (!ret && ((value ^ q->value) & mask))
			ret = -EIO;
	}
done:
	if (!ret && q->write && q->reg == 1)
		power_supply_changed(e->bq->charger);
	if (ret)
		energy_revoke(e);
	return ret;
}

static int energy_gpio(struct bq2415x_energy *e, struct rapuyama_energy_gpio *q)
{
	static const unsigned int types[] = {
		IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_EDGE_RISING,
		IRQ_TYPE_LEVEL_HIGH, IRQ_TYPE_LEVEL_LOW,
	};
	int ret = 0;
	bool armed;

	if (q->reserved[0] || q->reserved[1])
		return -EINVAL;
	if (!e->active || e->revoked)
		return -EOWNERDEAD;
	switch (q->operation) {
	case RAPUYAMA_ENERGY_GPIO_READ:
		ret = gpiod_get_raw_value_cansleep(e->stat);
		if (ret >= 0) { q->value = !!ret; ret = 0; }
		break;
	case RAPUYAMA_ENERGY_GPIO_CONFIG:
		if (q->value != 0x11)
			return -EINVAL;
		ret = gpiod_set_config(e->stat, pinconf_to_config_packed(RAPUYAMA_PIN_CONFIG_EM_STAT, 0x11));
		break;
	case RAPUYAMA_ENERGY_GPIO_TYPE:
		if (q->value >= ARRAY_SIZE(types))
			return -EINVAL;
		armed = atomic_read(&e->irq_enabled);
		energy_irq_control(e, false);
		ret = irq_set_irq_type(e->stat_irq, types[q->value]);
		if (!ret && armed)
			energy_irq_control(e, true);
		break;
	case RAPUYAMA_ENERGY_GPIO_ACK:
		ret = irq_set_irqchip_state(e->stat_irq, IRQCHIP_STATE_PENDING, false);
		if (!ret)
			atomic_set(&e->pending, 0);
		break;
	case RAPUYAMA_ENERGY_GPIO_ENABLE:
		energy_irq_control(e, true);
		break;
	case RAPUYAMA_ENERGY_GPIO_DISABLE:
		energy_irq_control(e, false);
		break;
	case RAPUYAMA_ENERGY_GPIO_EVENT:
		q->value = !!atomic_xchg(&e->pending, 0);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		energy_revoke(e);
	return ret;
}

static int energy_pmic(struct bq2415x_energy *e, struct rapuyama_energy_pmic *q)
{
	int ret;

	if (q->reserved || q->write > 1 || q->value > 0xffff || (!q->write && q->value))
		return -EINVAL;
	if (!e->active || e->revoked)
		return -EOWNERDEAD;
#if IS_REACHABLE(CONFIG_RAPUYAMA_WATCHDOG)
	ret = rapuyama_pmic_em_field(q->field, &q->value, q->write);
#else
	ret = -ENODEV;
#endif
	if (ret && ret != -EPERM && ret != -EINVAL)
		energy_revoke(e);
	return ret;
}

static int energy_acquire(struct bq2415x_energy *e)
{
	unsigned int chip;
	u8 identity;
	int ret;

	if (e->active || e->revoked)
		return -EBUSY;
	for (chip = 0; chip < 2; chip++) {
		ret = energy_read(e, chip, 3, &identity);
		if (ret || identity != (chip ? 0x41 : 0x51))
			return ret ? ret : -ENODEV;
	}
	mutex_lock(&bq2415x_i2c_mutex);
	WRITE_ONCE(e->bq->firmware_owned, true);
	mutex_unlock(&bq2415x_i2c_mutex);
	bq2415x_set_autotimer(e->bq, 0);
	WRITE_ONCE(e->active, true);
	energy_irq_control(e, false);
	atomic_set(&e->pending, 0);
	ret = energy_stop_all(e);
	for (chip = 0; !ret && chip < 2; chip++) {
		ret = energy_write(e, chip, 2, 30 << 2);
		if (!ret)
			ret = energy_write(e, chip, 4, 1);
		if (!ret)
			ret = energy_write(e, chip, 1, 6);
		if (!ret)
			ret = energy_limits(e, chip);
	}
	if (ret) {
		energy_revoke(e);
		return ret;
	}
	e->deadline = jiffies + ENERGY_LEASE;
	mod_delayed_work(system_wq, &e->guard, HZ);
	power_supply_changed(e->bq->charger);
	return 0;
}

static void energy_status(struct bq2415x_energy *e, struct rapuyama_energy_status *s)
{
	*s = (struct rapuyama_energy_status) {
		.version = RAPUYAMA_ENERGY_VERSION,
		.flags = (e->active ? RAPUYAMA_ENERGY_OWNED : 0) |
			(e->revoked ? RAPUYAMA_ENERGY_REVOKED : 0) | e->inhibit,
		.usb_budget_ua = e->budget,
	};
}

static long energy_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct bq2415x_energy *e = file->private_data;
	struct rapuyama_energy_status status;
	struct rapuyama_energy_io io;
	struct rapuyama_energy_gpio gpio;
	struct rapuyama_energy_pmic pmic;
	void __user *user = (void __user *)arg;
	int ret = 0;

	mutex_lock(&e->lock);
	if (!e->bq || e->stopping) {
		ret = -ENODEV;
		goto out;
	}
	if (e->active && !e->revoked && time_after_eq(jiffies, e->deadline))
		energy_revoke(e);
	switch (cmd) {
	case RAPUYAMA_ENERGY_ACQUIRE:
		if (copy_from_user(&status, user, sizeof(status))) { ret = -EFAULT; break; }
		if (status.version != RAPUYAMA_ENERGY_VERSION || status.flags || status.reserved || status.usb_budget_ua) { ret = -EINVAL; break; }
		ret = energy_acquire(e);
		if (ret)
			break;
		fallthrough;
	case RAPUYAMA_ENERGY_STATUS:
		energy_status(e, &status);
		if (copy_to_user(user, &status, sizeof(status))) {
			if (cmd == RAPUYAMA_ENERGY_ACQUIRE)
				energy_revoke(e);
			ret = -EFAULT;
		}
		break;
	case RAPUYAMA_ENERGY_IO:
		if (copy_from_user(&io, user, sizeof(io))) { ret = -EFAULT; break; }
		ret = energy_io(e, &io);
		if (!ret && copy_to_user(user, &io, sizeof(io))) { energy_revoke(e); ret = -EFAULT; }
		break;
	case RAPUYAMA_ENERGY_GPIO:
		if (copy_from_user(&gpio, user, sizeof(gpio))) { ret = -EFAULT; break; }
		ret = energy_gpio(e, &gpio);
		if (!ret && copy_to_user(user, &gpio, sizeof(gpio))) { energy_revoke(e); ret = -EFAULT; }
		break;
	case RAPUYAMA_ENERGY_PMIC:
		if (copy_from_user(&pmic, user, sizeof(pmic))) { ret = -EFAULT; break; }
		ret = energy_pmic(e, &pmic);
		if (!ret && copy_to_user(user, &pmic, sizeof(pmic))) { energy_revoke(e); ret = -EFAULT; }
		break;
	case RAPUYAMA_ENERGY_HEARTBEAT:
		if (!e->active || e->revoked) { ret = -EOWNERDEAD; break; }
		if (time_after_eq(jiffies, e->deadline)) { energy_revoke(e); ret = -EOWNERDEAD; break; }
		e->deadline = jiffies + ENERGY_LEASE;
		break;
	case RAPUYAMA_ENERGY_RELEASE:
		if (!e->active || e->revoked) { ret = -EOWNERDEAD; break; }
		energy_irq_control(e, false);
		atomic_set(&e->pending, 0);
		ret = energy_stop_all(e);
		if (ret) { energy_revoke(e); break; }
		mutex_lock(&bq2415x_i2c_mutex);
		WRITE_ONCE(e->bq->firmware_owned, false);
		mutex_unlock(&bq2415x_i2c_mutex);
		ret = bq2415x_set_defaults(e->bq);
		if (ret) {
			mutex_lock(&bq2415x_i2c_mutex);
			WRITE_ONCE(e->bq->firmware_owned, true);
			mutex_unlock(&bq2415x_i2c_mutex);
			energy_revoke(e);
			break;
		}
		WRITE_ONCE(e->active, false);
		bq2415x_set_autotimer(e->bq, 1);
		power_supply_changed(e->bq->charger);
		break;
	default:
		ret = -ENOTTY;
	}
out:
	mutex_unlock(&e->lock);
	return ret;
}

static void energy_free(struct kref *ref)
{
	kfree(container_of(ref, struct bq2415x_energy, ref));
}

static int energy_open(struct inode *inode, struct file *file)
{
	struct bq2415x_energy *e = container_of(file->private_data, struct bq2415x_energy, misc);
	int ret = 0;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	mutex_lock(&e->lock);
	if (!e->bq || e->stopping)
		ret = -ENODEV;
	else if (e->opened)
		ret = -EBUSY;
	else {
		e->opened = true;
		kref_get(&e->ref);
		file->private_data = e;
	}
	mutex_unlock(&e->lock);
	return ret;
}

static int energy_close(struct inode *inode, struct file *file)
{
	struct bq2415x_energy *e = file->private_data;

	mutex_lock(&e->lock);
	/* Only explicit RELEASE restores Linux; owner death never does. */
	if (e->bq && !e->stopping && e->active && !e->revoked)
		energy_revoke(e);
	e->opened = false;
	mutex_unlock(&e->lock);
	kref_put(&e->ref, energy_free);
	return 0;
}

static const struct file_operations energy_fops = {
	.owner = THIS_MODULE,
	.open = energy_open,
	.release = energy_close,
	.unlocked_ioctl = energy_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static int bq2415x_energy_property(struct bq2415x_device *bq,
				 enum power_supply_property prop, union power_supply_propval *val)
{
	struct bq2415x_energy *e = bq->energy;
	unsigned int chip;
	u8 status, control;
	int ret = 0, result = POWER_SUPPLY_STATUS_NOT_CHARGING;

	mutex_lock(&e->lock);
	if (e->bq && e->active && !e->revoked && time_after_eq(jiffies, e->deadline))
		energy_revoke(e);
	if (!e->bq || e->stopping || e->revoked || !e->active) {
		val->intval = prop == POWER_SUPPLY_PROP_STATUS ? POWER_SUPPLY_STATUS_UNKNOWN : 0;
		goto out;
	}
	for (chip = 0; chip < 2; chip++) {
		ret = energy_read(e, chip, 0, &status);
		if (!ret)
			ret = energy_read(e, chip, 1, &control);
		if (ret) { energy_revoke(e); break; }
		if (!(control & 6) && !(status & 7)) {
			if (((status >> 4) & 3) == 1)
				result = POWER_SUPPLY_STATUS_CHARGING;
			else if (((status >> 4) & 3) == 2 && result != POWER_SUPPLY_STATUS_CHARGING)
				result = POWER_SUPPLY_STATUS_FULL;
		}
	}
	val->intval = prop == POWER_SUPPLY_PROP_STATUS ? result :
		result == POWER_SUPPLY_STATUS_CHARGING || result == POWER_SUPPLY_STATUS_FULL;
out:
	mutex_unlock(&e->lock);
	return ret;
}

static int bq2415x_energy_init(struct bq2415x_device *bq)
{
	struct bq2415x_energy *e;
	struct i2c_client *client = to_i2c_client(bq->dev);
	int ret;

	if (!device_property_present(bq->dev, "nokia,em-control"))
		return 0;
	if (!of_machine_is_compatible("nokia,e6") || client->addr != 0x6b || !bq->battery || !bq->source)
		return -EINVAL;
	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;
	mutex_init(&e->lock);
	kref_init(&e->ref);
	INIT_DELAYED_WORK(&e->guard, energy_guard);
	atomic_set(&e->irq_enabled, 0);
	atomic_set(&e->pending, 0);
	e->dynamic = i2c_new_dummy_device(client->adapter, 0x6a);
	if (IS_ERR(e->dynamic)) { ret = PTR_ERR(e->dynamic); goto fail; }
	e->stat = gpiod_get_optional(bq->dev, "nokia,em-stat", GPIOD_ASIS);
	if (IS_ERR(e->stat)) { ret = PTR_ERR(e->stat); goto fail_dynamic; }
	if (!e->stat) { ret = -EINVAL; goto fail_dynamic; }
	if (gpiod_is_active_low(e->stat)) { ret = -EINVAL; goto fail_gpio; }
	e->stat_irq = gpiod_to_irq(e->stat);
	if (e->stat_irq < 0) { ret = e->stat_irq; goto fail_gpio; }
	ret = request_irq(e->stat_irq, energy_irq, IRQF_NO_AUTOEN, "rapuyama-energy", e);
	if (ret)
		goto fail_gpio;
	e->bq = bq;
	e->misc = (struct miscdevice) { .minor = MISC_DYNAMIC_MINOR,
		.name = "rapuyama-energy", .fops = &energy_fops, .mode = 0600, .parent = bq->dev };
	bq->energy = e;
	ret = misc_register(&e->misc);
	if (!ret)
		return 0;
	bq->energy = NULL;
	free_irq(e->stat_irq, e);
fail_gpio:
	gpiod_put(e->stat);
fail_dynamic:
	i2c_unregister_device(e->dynamic);
fail:
	kref_put(&e->ref, energy_free);
	return ret;
}

static void bq2415x_energy_shutdown(struct bq2415x_device *bq)
{
	struct bq2415x_energy *e = bq->energy;

	if (!e)
		return;
	mutex_lock(&e->lock);
	WRITE_ONCE(e->stopping, true);
	energy_irq_control(e, false);
	if (e->active)
		energy_revoke(e);
	mutex_unlock(&e->lock);
	cancel_delayed_work_sync(&e->guard);
}

static void bq2415x_energy_exit(struct bq2415x_device *bq)
{
	struct bq2415x_energy *e = bq->energy;

	if (!e)
		return;
	misc_deregister(&e->misc);
	bq2415x_energy_shutdown(bq);
	free_irq(e->stat_irq, e);
	mutex_lock(&e->lock);
	e->bq = NULL;
	mutex_unlock(&e->lock);
	gpiod_put(e->stat);
	i2c_unregister_device(e->dynamic);
}

static void bq2415x_energy_put(struct bq2415x_device *bq)
{
	if (bq->energy)
		kref_put(&bq->energy->ref, energy_free);
	bq->energy = NULL;
}
#else
static int bq2415x_energy_init(struct bq2415x_device *bq) { return 0; }
static void bq2415x_energy_notify(struct bq2415x_device *bq) { }
static void bq2415x_energy_exit(struct bq2415x_device *bq) { }
static void bq2415x_energy_put(struct bq2415x_device *bq) { }
static void bq2415x_energy_shutdown(struct bq2415x_device *bq) { }
static int bq2415x_energy_property(struct bq2415x_device *bq,
				 enum power_supply_property prop, union power_supply_propval *val)
{ return -ENODEV; }
#endif
