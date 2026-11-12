/* SPDX-License-Identifier: GPL-2.0 */
/* gazoo fields used by em (watch out for the watchdog!!!) */
static bool rapuyama_em_field_allowed(u32 field, bool write)
{
	switch (field) {
	case 0x018000: case 0x014000: case 0x010002:
	case 0x028000: case 0x024000: case 0x020002:
	case 0x210040: case 0x230040: case 0x210400: case 0x230400:
	case 0x2c0100: case 0x2c00ff: case 0x2400ff: case 0x09ffff:
		return true;
	case 0x01ffff: case 0x02ffff: case 0x21ffff: case 0x23ffff:
	case 0x220001: case 0x220800: case 0x2dffff: case 0x38ffff:
	case 0x242000: case 0x241000:
	case 0x240100: case 0x240e00:
		return !write;
	default:
		return false;
	}
}

int rapuyama_pmic_em_field(u32 field, u32 *value, bool write)
{
	struct rapuyama_wdt *rw = READ_ONCE(rapuyama_pmic);
	u32 bank = field >> 16, mask = field & 0xffff;
	u32 dev = bank >= 32 ? 2 : 1, reg = bank & 31;
	unsigned long flags;
	u16 old, desired, check;
	unsigned int shift = 0;
	int ret;
	bool acknowledge = reg == 1;

	if (!value)
		return -EINVAL;
	if (!rapuyama_em_field_allowed(field, write) ||
	    (write && (field == 0x2400ff || field == 0x09ffff) && *value))
		return -EPERM;
	while (!(mask & BIT(shift)))
		shift++;
	if (write && (*value & ~(mask >> shift)))
		return -EINVAL;
	if (!rw)
		return -EPROBE_DEFER;

	spin_lock_irqsave(&rw->lock, flags);
	if (write) {
		udelay(125);
		if (acknowledge) {
			desired = (*value << shift) & mask;
			ret = 0;
		} else {
			ret = __rapuyama_cbus_read(rw, dev, reg, &old);
			desired = ret ? 0 : (old & ~mask) | ((*value << shift) & mask);
		}
		if (!ret)
			ret = __rapuyama_cbus_write(rw, dev, reg, desired);
		udelay(125);
		if (!ret && !acknowledge) {
			ret = __rapuyama_cbus_read(rw, dev, reg, &check);
			if (!ret && ((check ^ desired) & mask))
				ret = -EIO;
		}
	} else {
		ret = __rapuyama_cbus_read(rw, dev, reg, &old);
		if (!ret)
			*value = (old & mask) >> shift;
	}
	spin_unlock_irqrestore(&rw->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(rapuyama_pmic_em_field);

int rapuyama_pmic_em_stop(void)
{
	static const struct { u32 field, value; } stop[] = {
		{0x028000, 1}, {0x024000, 1}, {0x020002, 1}, {0x230400, 1}, {0x230040, 1},
		{0x2c0100, 0}, {0x09ffff, 0}, {0x2400ff, 0},
	};
	unsigned int i;
	int ret = 0, error;
	u32 value;

	for (i = 0; i < ARRAY_SIZE(stop); i++) {
		value = stop[i].value;
		error = rapuyama_pmic_em_field(stop[i].field, &value, true);
		if (!ret)
			ret = error;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(rapuyama_pmic_em_stop);