// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E5-00 light pwm on the gazoo pmic (cbus, rapuyama_wdt.c owns it).
 *
 * E5 isasw hal_light_phy_gazoo.c, 0xc633f0d2(ch, level 0..10000):
 * pwm = level * 127 / 10000 into gazoo field reg << 16 | mask:
 *   ch 0: 0x257f00 = dev 2 reg 5 bits 14:8 (+ mode bits 4 and 0)
 *   ch 1: 0x2a007f = dev 2 reg 0xa bits 6:0 (+ mode bit 8)
 * the screen is not on these (P1 run: pwm0 changed, the screen did not; the
 * screen is the panel DCS backlight in rapuyama-e5-fb.c). pwm0 = keyboard
 * light (seen on the device: 0 off, 10 dim, 127 full); pwm1 (0 at handoff)
 * lights nothing visible. only the pwm field is written; the mode bits stay
 * as symbian left them. with no fb tied to them, the backlight core blanks
 * both together with fb0 (screen off = key light off)
 */
#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/rapuyama-pmic.h>

#define GAZOO_DEV	2
#define PWM_MAX		127

struct ry_pwm {
	u32 reg;
	u16 mask;
	unsigned int shift;
};

static const struct ry_pwm ry_pwm_ch[2] = {
	{ .reg = 0x05, .mask = 0x7f00, .shift = 8 },
	{ .reg = 0x0a, .mask = 0x007f, .shift = 0 },
};

static int ry_bl_update(struct backlight_device *bl)
{
	const struct ry_pwm *p = bl_get_data(bl);

	return rapuyama_pmic_update(GAZOO_DEV, p->reg, p->mask,
				    backlight_get_brightness(bl) << p->shift, NULL);
}

static const struct backlight_ops ry_bl_ops = {
	.update_status = ry_bl_update,
};

static int ry_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = PWM_MAX,
	};
	struct backlight_device *bl;
	const struct ry_pwm *p;
	char name[16];
	u32 ch;
	u16 v;
	int ret;

	if (of_property_read_u32(pdev->dev.of_node, "nokia,pwm-channel", &ch) ||
	    ch >= ARRAY_SIZE(ry_pwm_ch))
		return dev_err_probe(&pdev->dev, -EINVAL, "bad nokia,pwm-channel\n");
	p = &ry_pwm_ch[ch];
	/* read only: keep symbian's level as the start value */
	ret = rapuyama_pmic_update(GAZOO_DEV, p->reg, 0, 0, &v);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "no pmic\n");
	props.brightness = (v & p->mask) >> p->shift;
	snprintf(name, sizeof(name), "gazoo-pwm%u", ch);
	bl = devm_backlight_device_register(&pdev->dev, name, &pdev->dev,
					   (void *)p, &ry_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);
	dev_info(&pdev->dev, "%s: gazoo 2.%02x = %04x, level %d\n", name,
		 p->reg, v, props.brightness);
	return 0;
}

static const struct of_device_id ry_bl_of_match[] = {
	{ .compatible = "nokia,rapuyama-e5-light-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, ry_bl_of_match);

static struct platform_driver ry_bl_driver = {
	.probe = ry_bl_probe,
	.driver = {
		.name = "rapuyama-e5-bl",
		.of_match_table = ry_bl_of_match,
	},
};
module_platform_driver(ry_bl_driver);

MODULE_DESCRIPTION("Nokia E5 gazoo light pwm");
MODULE_LICENSE("GPL");
