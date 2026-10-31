// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nokia E5-00 (RapuYama) QWERTY keyboard: 8x8 GPIO matrix, polled.
 *
 * Copies the scan of the E5 ROM's ekeyb.dll (0x80157d78 / 0x80157e28):
 * column lines (outputs, gpio 0 3 13 33 36 38 45 55) are all driven high
 * except the scanned one, which is driven low; a pressed key pulls its row
 * line (gpio 114..119 121 2) to 0. The row level is read from the gpio irq
 * block's RAW state register, which uses the egpio "logical" pin number
 * (gpio >= 106 -> gpio - 18). The plain gpio input register does NOT show
 * these pins (checked on the device, K1/K2 2026-10-31).
 *
 * Pin directions are left as Symbian set them (columns outputs, all low at
 * rest; rows inputs). Nothing else in the gpio block is written: only the
 * set/clear registers of the eight column pins. At rest all columns are low
 * again, so an idle check costs two register reads.
 *
 * Without a "linux,keymap" property every key is reported as MSC_SCAN only,
 * and the learn parameter logs "key rN cM down/up" to find the layout.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/workqueue.h>

#define E5KP_ROWS	8
#define E5KP_COLS	8
#define E5KP_ROW_SHIFT	3

/* gpio block (phys 0x20031900): output set / clear, bank k */
#define GPIO_OUT_SET(k)	(0x94 + 0x10 * (k))
#define GPIO_OUT_CLR(k)	(0x98 + 0x10 * (k))
/* direction, write 1 = that pin becomes input / output (egpio, K0) */
#define GPIO_DIR_IN(k)	(0xf4 + 0x10 * (k))
#define GPIO_DIR_OUT(k)	(0xf8 + 0x10 * (k))
/* irq block (phys 0x20033400): raw pin state, logical bank k */
#define IRQ_RAW(k)	(0x40 * (k) + 0x08)

/* ekeyb.dll defaults, RAM copies c8009c34 (rows) / c8009c54 (cols) */
static const u8 e5kp_row_gpio[E5KP_ROWS] = { 114, 115, 116, 117, 118, 119, 121, 2 };
static const u8 e5kp_col_gpio[E5KP_COLS] = { 0, 3, 13, 33, 36, 38, 45, 55 };

static bool learn;
module_param(learn, bool, 0644);
MODULE_PARM_DESC(learn, "log every key change as row/column (default off)");

/* two keys on one row tie a low and a high column together when the idle
 * columns are driven high: the row then reads wrong (sym+@ on r0). hiz=1
 * floats the idle columns (input) instead, only the scanned one drives low
 */
static bool hiz = true;
module_param(hiz, bool, 0644);
MODULE_PARM_DESC(hiz, "float idle columns while scanning (default on, 0 = drive them high)");

static unsigned int poll_ms = 20;
module_param(poll_ms, uint, 0644);
MODULE_PARM_DESC(poll_ms, "scan interval in ms (default 20)");

static unsigned int col_delay_us = 5;
module_param(col_delay_us, uint, 0644);
MODULE_PARM_DESC(col_delay_us, "settle time after driving a column, us (default 5)");

struct e5kp {
	struct device *dev;
	struct input_dev *input;
	void __iomem *gpio;
	void __iomem *irq;
	struct delayed_work work;

	u32 col_mask[2];		/* column bits in gpio banks 0 and 1 */
	u8 col_bank[E5KP_COLS];
	u32 col_bit[E5KP_COLS];
	u8 row_bank[E5KP_ROWS];		/* logical irq-block bank: 0 or 3 */
	u32 row_bit[E5KP_ROWS];

	u8 state[E5KP_COLS];		/* reported: bit r of state[c] = pressed */
	u8 last[E5KP_COLS];		/* previous raw scan, for debouncing */
};

static void e5kp_cols_dir(struct e5kp *kp, bool out)
{
	int k;

	for (k = 0; k < 2; k++)
		writel(kp->col_mask[k], kp->gpio +
			       (out ? GPIO_DIR_OUT(k) : GPIO_DIR_IN(k)));
}

static void e5kp_cols_level(struct e5kp *kp, bool high)
{
	int k;

	for (k = 0; k < 2; k++)
		writel(kp->col_mask[k], kp->gpio +
			       (high ? GPIO_OUT_SET(k) : GPIO_OUT_CLR(k)));
}

/* bit r set = row r reads 0 (pressed) */
static u8 e5kp_rows(struct e5kp *kp)
{
	u32 raw0 = readl_relaxed(kp->irq + IRQ_RAW(0));
	u32 raw3 = readl_relaxed(kp->irq + IRQ_RAW(3));
	u8 rows = 0;
	int r;

	for (r = 0; r < E5KP_ROWS; r++)
		if (!((kp->row_bank[r] ? raw3 : raw0) & kp->row_bit[r]))
			rows |= BIT(r);
	return rows;
}

static void e5kp_scan(struct e5kp *kp, u8 *cols)
{
	int c;

	memset(cols, 0, E5KP_COLS);
	/* at rest all columns are low: any key pulls its row down */
	if (!e5kp_rows(kp))
		return;

	if (hiz) {
		/* columns keep their low output latch, only the direction moves */
		e5kp_cols_dir(kp, false);
		for (c = 0; c < E5KP_COLS; c++) {
			void __iomem *g = kp->gpio;

			writel(kp->col_bit[c], g + GPIO_DIR_OUT(kp->col_bank[c]));
			udelay(col_delay_us);
			cols[c] = e5kp_rows(kp);
			writel(kp->col_bit[c], g + GPIO_DIR_IN(kp->col_bank[c]));
		}
		e5kp_cols_dir(kp, true);
		return;
	}

	for (c = 0; c < E5KP_COLS; c++) {
		e5kp_cols_level(kp, true);
		writel(kp->col_bit[c], kp->gpio + GPIO_OUT_CLR(kp->col_bank[c]));
		udelay(col_delay_us);
		cols[c] = e5kp_rows(kp);
	}
	e5kp_cols_level(kp, false);
}

static void e5kp_work(struct work_struct *work)
{
	struct e5kp *kp = container_of(to_delayed_work(work), struct e5kp, work);
	const unsigned short *keycode = kp->input->keycode;
	u8 cols[E5KP_COLS];
	bool sync = false;
	int c, r;

	e5kp_scan(kp, cols);

	/* accept a change only when two scans in a row agree */
	for (c = 0; c < E5KP_COLS; c++) {
		u8 stable = ~(cols[c] ^ kp->last[c]);
		u8 changed = (cols[c] ^ kp->state[c]) & stable;

		kp->last[c] = cols[c];
		for (r = 0; r < E5KP_ROWS; r++) {
			unsigned int code = MATRIX_SCAN_CODE(r, c, E5KP_ROW_SHIFT);
			bool down = cols[c] & BIT(r);

			if (!(changed & BIT(r)))
				continue;
			if (learn)
				dev_info(kp->dev, "key r%d c%d %s (keycode %u)\n",
					 r, c, down ? "down" : "up", keycode[code]);
			input_event(kp->input, EV_MSC, MSC_SCAN, code);
			input_report_key(kp->input, keycode[code], down);
			sync = true;
		}
		kp->state[c] ^= changed;
	}
	if (sync)
		input_sync(kp->input);

	schedule_delayed_work(&kp->work, msecs_to_jiffies(poll_ms));
}

static void e5kp_stop(void *data)
{
	struct e5kp *kp = data;

	cancel_delayed_work_sync(&kp->work);
	e5kp_cols_level(kp, false);
}

static int e5kp_probe(struct platform_device *pdev)
{
	static const struct matrix_keymap_data no_keymap = { .keymap_size = 0 };
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct e5kp *kp;
	int i, ret;

	kp = devm_kzalloc(dev, sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;
	kp->dev = dev;

	/* plain ioremap: the irq block range is also listed by the intc node */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	kp->gpio = devm_ioremap(dev, res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -EINVAL;
	kp->irq = devm_ioremap(dev, res->start, resource_size(res));
	if (!kp->gpio || !kp->irq)
		return -ENOMEM;

	for (i = 0; i < E5KP_COLS; i++) {
		unsigned int g = e5kp_col_gpio[i];

		kp->col_bank[i] = g / 32;
		kp->col_bit[i] = BIT(g % 32);
		kp->col_mask[g / 32] |= BIT(g % 32);
	}
	for (i = 0; i < E5KP_ROWS; i++) {
		unsigned int l = e5kp_row_gpio[i];

		if (l >= 106)
			l -= 18;
		kp->row_bank[i] = l / 32 ? 1 : 0;	/* 0 -> bank 0, 1 -> bank 3 */
		kp->row_bit[i] = BIT(l % 32);
		if (l / 32 != 0 && l / 32 != 3)
			return dev_err_probe(dev, -EINVAL, "row %d: bank %u\n", i, l / 32);
	}

	kp->input = devm_input_allocate_device(dev);
	if (!kp->input)
		return -ENOMEM;
	kp->input->name = "Nokia E5 keyboard";
	kp->input->phys = "rapuyama-e5-keypad/input0";
	kp->input->id.bustype = BUS_HOST;

	ret = matrix_keypad_build_keymap(device_property_present(dev, "linux,keymap") ?
					 NULL : &no_keymap, NULL,
					 E5KP_ROWS, E5KP_COLS, NULL, kp->input);
	if (ret)
		return dev_err_probe(dev, ret, "keymap\n");
	input_set_capability(kp->input, EV_MSC, MSC_SCAN);
	__set_bit(EV_REP, kp->input->evbit);

	ret = input_register_device(kp->input);
	if (ret)
		return ret;

	/* scan always runs (learn mode needs it without any input reader) */
	INIT_DELAYED_WORK(&kp->work, e5kp_work);
	ret = devm_add_action_or_reset(dev, e5kp_stop, kp);
	if (ret)
		return ret;
	schedule_delayed_work(&kp->work, 0);

	dev_info(dev, "8x8 matrix, polled %u ms, %s keymap, learn %d\n", poll_ms,
		 device_property_present(dev, "linux,keymap") ? "dt" : "no", learn);
	return 0;
}

static const struct of_device_id e5kp_of_match[] = {
	{ .compatible = "nokia,rapuyama-e5-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, e5kp_of_match);

static struct platform_driver e5kp_driver = {
	.probe	= e5kp_probe,
	.driver	= {
		.name		= "rapuyama-e5-keypad",
		.of_match_table	= e5kp_of_match,
	},
};
module_platform_driver(e5kp_driver);

MODULE_DESCRIPTION("Nokia E5-00 RapuYama polled keyboard matrix");
MODULE_LICENSE("GPL");
