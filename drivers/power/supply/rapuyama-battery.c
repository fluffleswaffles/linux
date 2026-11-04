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
 *
 * E5-00 ("nokia,rapuyama-e5-battery"): same isasw em stack, same hw_ad
 * channel table (0xc66775d8) and measurement switch (0xc6333424); its
 * scaler 0xc6334294 run on the E5's own calibration (data 0xc682ac5c, xor
 * 0xaa backup at 0xc682ae9c matches) gives
 *
 *   hw 8  battery voltage  mV = 2.04825 * raw + 2674.4 (within 2 mV)
 *   hw 2  battery ntc      kelvin, e5_btemp_k below
 *
 * and its battery monitor (em_batmon5.c) gives the E5 capacity curve and
 * the low battery poweroff, see e5_ocv and e5_cal.off_mv
 *
 * E5 battery current (em_hal_me_ibat.c, em_hal_gazoo 0xc63335a2): the gazoo
 * averages it in hardware. dev 2 reg 0x0c bits 7:0 = window in 250 ms units
 * (symbian: 20 = 5 s), dev 2 reg 0x0d = signed sum over the window, 0x7fff /
 * -0x8000 = no result. isasw: mA = -(sum / units) * 0.8030941611842105, i.e.
 * positive = drawn from the battery; here negative = discharging.
 * reg 0x0c bit 8 enables it (isasw 0xc63301c4 sets it after the window);
 * on the E5 it then runs continuously and 0x0d holds the last finished
 * window. symbian leaves it on or off and at 1 or 20 units, so linux sets
 * the 5 s window and enables it at probe
 */
#include <linux/devm-helpers.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/soc/rapuyama-pmic.h>
#include <linux/soc/rapuyama-usb.h>
#include <linux/workqueue.h>

#define ADC_VBAT	8
#define ADC_BTEMP	2
#define ADC_VCHAR	9
#define VCHAR_MIN	50	/* raw, a charger is in */
#define VBAT_SAMPLES	4

#define IBAT_DEV	2
#define IBAT_WINDOW	0x0c	/* bits 7:0, 250 ms units */
#define IBAT_ON		BIT(8)
#define IBAT_UNITS	20	/* symbian's normal 5 s */
#define IBAT_SUM	0x0d
#define IBAT_NA_PER_LSB	803094	/* 0.8030941611842105 mA */

/* isasw's scaler at raw 0, 16, 32 ... 1008 and 1023, kelvin */
static const u16 btemp_k[] = {
	0, 405, 375, 359, 350, 342, 336, 331, 327, 323, 320, 317, 315, 312,
	310, 308, 306, 304, 302, 300, 298, 297, 295, 294, 292, 291, 290, 288,
	287, 285, 284, 283, 282, 280, 279, 278, 277, 276, 274, 273, 272, 271,
	269, 268, 267, 265, 264, 263, 261, 260, 258, 257, 255, 254, 252, 250,
	248, 245, 243, 240, 236, 232, 226, 217, 173,
};

static const u16 e5_btemp_k[] = {
	0, 383, 365, 353, 345, 338, 333, 329, 325, 322, 319, 316, 313, 311,
	309, 307, 305, 303, 301, 299, 298, 296, 295, 293, 292, 290, 289, 288,
	286, 285, 284, 283, 281, 280, 279, 278, 276, 275, 274, 273, 272, 270,
	269, 268, 267, 265, 264, 263, 261, 260, 258, 257, 255, 253, 251, 250,
	247, 245, 243, 240, 236, 232, 226, 217, 181,
};

/*
 * vbat mV = (raw * mul + add) / 10000, ntc kelvin per raw / 16.
 * no_charging: linux never charges this battery (E5: the bq2415x is not
 * programmed, charging stays in symbian), so a plugged cable is not a
 * reason to let the percentage rise: it only follows the voltage down, and
 * catches up in bigger steps when it is more than 5 % off (start value read right
 * after symbian charged, voltage still raised)
 */
struct ry_ocv {
	u16 mv;
	u8 pct;
};

/* rough open circuit voltage curve of a li-ion cell, mV -> % */
static const struct ry_ocv ocv[] = {
	{ 3300, 0 }, { 3500, 5 }, { 3600, 10 }, { 3700, 30 }, { 3750, 45 },
	{ 3800, 55 }, { 3850, 65 }, { 3900, 72 }, { 4000, 85 }, { 4100, 95 },
	{ 4200, 100 },
};

/*
 * E5 isasw em_batmon5.c: the battery table at 0xc6666770 (copied into the
 * live monitor state at 0xc682b328, same values there), 19 x {used mAh, mV}
 * from 0 mAh at 4179 mV to 1052 mAh at 3100 mV (BL-4D, 1200 mAh nominal,
 * 0xc66667c0). % = 100 * (1052 - used) / 1052, rounded
 */
static const struct ry_ocv e5_ocv[] = {
	{ 3240, 0 }, { 3319, 1 }, { 3395, 1 }, { 3479, 2 }, { 3566, 3 },
	{ 3636, 5 }, { 3662, 6 }, { 3681, 13 }, { 3716, 17 }, { 3750, 24 },
	{ 3776, 38 }, { 3816, 53 }, { 3853, 58 }, { 3948, 74 }, { 4052, 88 },
	{ 4158, 99 }, { 4179, 100 },
};

/*
 * off_mv: power off once the averaged voltage stays below it for OFF_POLLS
 * polls (0 = never). E5: 3300 mV = em_batmon5's low limit (default
 * 0xce4 at 0xc6335888, live 0xc682b1ae = 3300, setter 0xc6335f98 refuses
 * anything lower); it also wants 3400 mV at boot (0xc6335a0e)
 */
struct ry_bat_cal {
	u32 vbat_mul, vbat_add;
	bool no_charging;
	const u16 *btemp_k;
	unsigned int btemp_n;
	const struct ry_ocv *ocv;
	unsigned int ocv_n;
	u16 off_mv;
	bool ibat;	/* gazoo current meter, E5 only so far */
};

static const struct ry_bat_cal e6_cal = {
	.vbat_mul = 20637, .vbat_add = 27272000,
	.btemp_k = btemp_k, .btemp_n = ARRAY_SIZE(btemp_k),
	.ocv = ocv, .ocv_n = ARRAY_SIZE(ocv),
};

static const struct ry_bat_cal e5_cal = {
	.vbat_mul = 20483, .vbat_add = 26744277,
	.no_charging = true,
	.btemp_k = e5_btemp_k, .btemp_n = ARRAY_SIZE(e5_btemp_k),
	.ocv = e5_ocv, .ocv_n = ARRAY_SIZE(e5_ocv),
	.off_mv = 3300,
	.ibat = true,
};

static const struct ry_bat_cal *cal = &e6_cal;

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
	*mv = (sum * cal->vbat_mul / VBAT_SAMPLES + cal->vbat_add) / 10000;
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
	if (i + 1 >= cal->btemp_n) {
		k10 = cal->btemp_k[cal->btemp_n - 1] * 10;
	} else {
		k10 = cal->btemp_k[i] * 10 -
		      ((int)cal->btemp_k[i] - cal->btemp_k[i + 1]) * 10 * (int)f / 16;
	}
	*dc = k10 - 2732;
	return 0;
}

/* average over the meter's window, uA, negative = discharging */
static int ry_bat_current(int *ua)
{
	u16 win, raw;
	unsigned int units;
	s16 sum;
	int ret;

	ret = rapuyama_pmic_update(IBAT_DEV, IBAT_WINDOW, 0, 0, &win);
	if (!ret)
		ret = rapuyama_pmic_update(IBAT_DEV, IBAT_SUM, 0, 0, &raw);
	if (ret)
		return ret;
	units = win & 0xff;
	sum = raw;
	if (sum == 0x7fff || sum == -0x8000)
		return -ENODATA;
	/* isasw counts a window of 0 as 1 */
	*ua = div_s64((s64)sum * IBAT_NA_PER_LSB, (units ?: 1) * 1000);
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
	const struct ry_ocv *ocv = cal->ocv;
	unsigned int i;

	if (mv <= ocv[0].mv)
		return 0;
	for (i = 1; i < cal->ocv_n; i++)
		if (mv < ocv[i].mv)
			return ocv[i - 1].pct + (ocv[i].pct - ocv[i - 1].pct) *
			       (mv - ocv[i - 1].mv) / (ocv[i].mv - ocv[i - 1].mv);
	return 100;
}

#define PCT_HOLD	60
#define OFF_POLLS	30	/* ~30 s below off_mv */

struct ry_bat {
	struct power_supply *bat, *usb;
	struct delayed_work poll;
	int charger;
	int mv16;	/* vbat averaged over ~16 s, mV * 16 */
	int pct, hold;
	int low;	/* polls in a row below cal->off_mv */
};


static void ry_bat_track(struct ry_bat *rb)
{
	int mv, want;

	if (ry_bat_vbat_mv(&mv))
		return;
	rb->mv16 += mv - rb->mv16 / 16;
	if (cal->off_mv && rb->mv16 / 16 < cal->off_mv) {
		if (++rb->low == OFF_POLLS) {
			dev_crit(&rb->bat->dev, "battery empty (%d mV < %u mV), powering off\n",
				 rb->mv16 / 16, cal->off_mv);
			orderly_poweroff(true);
		}
	} else {
		rb->low = 0;
	}
	want = ry_bat_capacity(rb->mv16 / 16);
	if (cal->no_charging) {
		/* more than 5 % off: close half the gap now, no hold */
		if (rb->pct - want > 5) {
			rb->pct -= (rb->pct - want) / 2;
			rb->hold = PCT_HOLD;
			power_supply_changed(rb->bat);
			return;
		}
		if (rb->hold > 0) {
			rb->hold--;
			return;
		}
		if (want < rb->pct) {
			rb->pct--;
			rb->hold = PCT_HOLD;
			power_supply_changed(rb->bat);
		}
		return;
	}
	if (rb->hold > 0) {
		rb->hold--;
		return;
	}
	if ((rb->charger && want > rb->pct) || (!rb->charger && want < rb->pct)) {
		rb->pct += want > rb->pct ? 1 : -1;
		rb->hold = PCT_HOLD;
		power_supply_changed(rb->bat);
	}
}

/* the charger has no irq: watch vchar so a plug reaches userspace within a second */
static void ry_bat_poll(struct work_struct *work)
{
	struct ry_bat *rb = container_of(to_delayed_work(work), struct ry_bat, poll);
	int in = ry_bat_charger();

	ry_bat_track(rb);
	if (in != rb->charger) {
		rb->charger = in;
		power_supply_changed(rb->usb);
		power_supply_changed(rb->bat);
	}
	schedule_delayed_work(&rb->poll, HZ);
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
		val->intval = ((struct ry_bat *)power_supply_get_drvdata(psy))->pct;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = ry_bat_temp(&val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!cal->ibat)
			return -EINVAL;
		ret = ry_bat_current(&val->intval);
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
	POWER_SUPPLY_PROP_CURRENT_NOW,	/* last: left out without cal->ibat */
};

static struct power_supply_desc ry_bat_desc = {
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
	struct ry_bat *rb;
	int mv, dc, ret;

	rb = devm_kzalloc(&pdev->dev, sizeof(*rb), GFP_KERNEL);
	if (!rb)
		return -ENOMEM;
	cal = of_device_get_match_data(&pdev->dev);
	ret = ry_bat_vbat_mv(&mv);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "no pmic adc\n");
	rb->charger = ry_bat_charger();
	rb->mv16 = mv * 16;
	rb->pct = ry_bat_capacity(mv);
	if (!cal->ibat)
		ry_bat_desc.num_properties = ARRAY_SIZE(ry_bat_props) - 1;
	cfg.drv_data = rb;
	rb->bat = devm_power_supply_register(&pdev->dev, &ry_bat_desc, &cfg);
	if (IS_ERR(rb->bat))
		return PTR_ERR(rb->bat);
	usb_cfg.of_node = of_get_child_by_name(pdev->dev.of_node, "usb");
	ret = devm_add_action_or_reset(&pdev->dev, ry_bat_put_node, usb_cfg.of_node);
	if (ret)
		return ret;
	rb->usb = devm_power_supply_register(&pdev->dev, &ry_usb_desc, &usb_cfg);
	if (IS_ERR(rb->usb))
		return PTR_ERR(rb->usb);
	ret = devm_delayed_work_autocancel(&pdev->dev, &rb->poll, ry_bat_poll);
	if (ret)
		return ret;
	schedule_delayed_work(&rb->poll, HZ);
	if (ry_bat_temp(&dc))
		dev_info(&pdev->dev, "%d mV, no temperature, charger %s\n", mv,
			 ry_bat_charger() ? "in" : "out");
	else
		dev_info(&pdev->dev, "%d mV, %d.%d C, charger %s\n", mv,
			 dc / 10, abs(dc % 10), ry_bat_charger() ? "in" : "out");
	if (cal->ibat) {
		u16 old = 0;

		ret = rapuyama_pmic_update(IBAT_DEV, IBAT_WINDOW, IBAT_ON | 0xff,
					   IBAT_ON | IBAT_UNITS, &old);
		dev_info(&pdev->dev, "current meter: was 0x%04x, now %u x 250 ms, on (%d)\n",
			 old, IBAT_UNITS, ret);
	}
	return 0;
}

static const struct of_device_id ry_bat_of_match[] = {
	{ .compatible = "nokia,rapuyama-battery", .data = &e6_cal },
	{ .compatible = "nokia,rapuyama-e5-battery", .data = &e5_cal },
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