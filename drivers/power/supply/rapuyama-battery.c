// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E6 battery, from the gazoo pmic adc (rapuyama_wdt.c owns the cbus).
 *
 * Symbian does all of this in ISASW's energy management (em_hal_*.c), which
 * doesn't run under Linux. Channels and scaling come from it: the hw channel
 * per measurement (em_hal_gazoo FUN_c62271be + the hw_ad table at
 * 0xc65d5d88) and the result of its own scaler (em_hal_adc_cal FUN_c622420c)
 * run on this phone's calibration (guest ram 0xc68127a4):
 *
 *   hw 8  battery voltage  mV = 2.0637 * raw + 2727 (within 6 mV of isasw)
 *   hw 2  battery ntc      kelvin, table below (raw / 16)
 *   hw 9  charger voltage  0 without a charger, ~248 on usb
 *
 * the optional charger phandle supplies charging status
 */
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/soc/rapuyama-pmic.h>
#include <linux/soc/rapuyama-usb.h>

#define ADC_VBAT	8
#define ADC_BTEMP	2
#define ADC_VCHAR	9
#define VCHAR_MIN	50	/* raw, a charger is in */
#define VBAT_SAMPLES	4

/* isasw's scaler at raw 0, 16, 32 ... 1008 and 1023, kelvin */
static const u16 btemp_k[] = {
	0, 405, 375, 359, 350, 342, 336, 331, 327, 323, 320, 317, 315, 312,
	310, 308, 306, 304, 302, 300, 298, 297, 295, 294, 292, 291, 290, 288,
	287, 285, 284, 283, 282, 280, 279, 278, 277, 276, 274, 273, 272, 271,
	269, 268, 267, 265, 264, 263, 261, 260, 258, 257, 255, 254, 252, 250,
	248, 245, 243, 240, 236, 232, 226, 217, 173,
};

/* rough open circuit voltage curve of a li-ion cell, mV -> % */
static const struct {
	u16 mv;
	u8 pct;
} ocv[] = {
	{ 3300, 0 }, { 3500, 5 }, { 3600, 10 }, { 3700, 30 }, { 3750, 45 },
	{ 3800, 55 }, { 3850, 65 }, { 3900, 72 }, { 4000, 85 }, { 4100, 95 },
	{ 4200, 100 },
};

static int ry_bat_vbat_mv(int *mv)
{
	u32 sum = 0;
	u16 raw;
	int i, ret;

	for (i = 0; i < VBAT_SAMPLES; i++) {
		ret = rapuyama_pmic_adc(ADC_VBAT, &raw);
		if (ret)
			return ret;
		sum += raw;
	}
	/* 2.0637 mV per step + 2727.2 mV, in 1/10000 mV */
	*mv = (sum * 20637 / VBAT_SAMPLES + 27272000) / 10000;
	return 0;
}

/* tenths of a degree */
static int ry_bat_temp(int *dc)
{
	unsigned int i, f;
	u16 raw;
	int k10, ret;

	ret = rapuyama_pmic_adc(ADC_BTEMP, &raw);
	if (ret)
		return ret;
	/* 0 = no ntc, isasw says 0 K there */
	if (raw < 16)
		return -ENODATA;
	i = raw / 16;
	f = raw % 16;
	if (i + 1 >= ARRAY_SIZE(btemp_k)) {
		k10 = btemp_k[ARRAY_SIZE(btemp_k) - 1] * 10;
	} else {
		k10 = btemp_k[i] * 10 -
		      ((int)btemp_k[i] - btemp_k[i + 1]) * 10 * (int)f / 16;
	}
	*dc = k10 - 2732;
	return 0;
}

static int ry_bat_charger(void)
{
	u16 raw;

	if (rapuyama_pmic_adc(ADC_VCHAR, &raw))
		return 0;
	return raw >= VCHAR_MIN;
}

static int ry_bat_capacity(int mv)
{
	unsigned int i;

	if (mv <= ocv[0].mv)
		return 0;
	for (i = 1; i < ARRAY_SIZE(ocv); i++)
		if (mv < ocv[i].mv)
			return ocv[i - 1].pct + (ocv[i].pct - ocv[i - 1].pct) *
			       (mv - ocv[i - 1].mv) / (ocv[i].mv - ocv[i - 1].mv);
	return 100;
}

static int ry_bat_get(struct power_supply *psy, enum power_supply_property psp,
		      union power_supply_propval *val)
{
	int ret = 0, mv = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_STATUS: {
		struct power_supply *charger;

		val->intval = ry_bat_charger() ? POWER_SUPPLY_STATUS_NOT_CHARGING :
						 POWER_SUPPLY_STATUS_DISCHARGING;
		charger = power_supply_get_by_phandle(psy->of_node, "nokia,charger");
		if (!IS_ERR_OR_NULL(charger)) {
			union power_supply_propval status;

			if (ry_bat_charger() && !power_supply_get_property(charger,
						POWER_SUPPLY_PROP_STATUS, &status))
				*val = status;
			power_supply_put(charger);
		}
		break;
	}
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = ry_bat_vbat_mv(&mv);
		val->intval = mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = ry_bat_vbat_mv(&mv);
		val->intval = ry_bat_capacity(mv);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = ry_bat_temp(&val->intval);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static const enum power_supply_property ry_bat_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc ry_bat_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = ry_bat_props,
	.num_properties = ARRAY_SIZE(ry_bat_props),
	.get_property = ry_bat_get,
};

static int ry_usb_get(struct power_supply *psy, enum power_supply_property psp,
		      union power_supply_propval *val)
{
	if (psp == POWER_SUPPLY_PROP_CURRENT_MAX) {
		val->intval = ry_bat_charger() ? rapuyama_usb_current_ma() * 1000 : 0;
		return 0;
	}
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;
	val->intval = ry_bat_charger();
	return 0;
}

static const enum power_supply_property ry_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc ry_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ry_usb_props,
	.num_properties = ARRAY_SIZE(ry_usb_props),
	.get_property = ry_usb_get,
};

static void ry_bat_put_node(void *node)
{
	of_node_put(node);
}

static int ry_bat_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = { .of_node = pdev->dev.of_node };
	struct power_supply_config usb_cfg = { };
	struct power_supply *psy;
	int mv, dc, ret;

	ret = ry_bat_vbat_mv(&mv);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "no pmic adc\n");
	psy = devm_power_supply_register(&pdev->dev, &ry_bat_desc, &cfg);
	if (IS_ERR(psy))
		return PTR_ERR(psy);
	usb_cfg.of_node = of_get_child_by_name(pdev->dev.of_node, "usb");
	ret = devm_add_action_or_reset(&pdev->dev, ry_bat_put_node, usb_cfg.of_node);
	if (ret)
		return ret;
	psy = devm_power_supply_register(&pdev->dev, &ry_usb_desc, &usb_cfg);
	if (IS_ERR(psy))
		return PTR_ERR(psy);
	if (ry_bat_temp(&dc))
		dev_info(&pdev->dev, "%d mV, no temperature, charger %s\n", mv,
			 ry_bat_charger() ? "in" : "out");
	else
		dev_info(&pdev->dev, "%d mV, %d.%d C, charger %s\n", mv,
			 dc / 10, abs(dc % 10), ry_bat_charger() ? "in" : "out");
	return 0;
}

static const struct of_device_id ry_bat_of_match[] = {
	{ .compatible = "nokia,rapuyama-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, ry_bat_of_match);

static struct platform_driver ry_bat_driver = {
	.probe = ry_bat_probe,
	.driver = {
		.name = "rapuyama-battery",
		.of_match_table = ry_bat_of_match,
	},
};
module_platform_driver(ry_bat_driver);

MODULE_DESCRIPTION("Nokia E6 (RapuYama) battery, gazoo adc");
MODULE_LICENSE("GPL");