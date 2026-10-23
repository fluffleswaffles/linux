// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E6 keypad: Toshiba TC3589x (id 0x80 = 3, version 2) on i2c 0x44
 *
 * The mainline tc3589x mfd + keypad drivers reset and reprogram the keypad
 * module, after that this chip NACKs every keypad register (KBDIC/KBDMSK,
 * even writes). So this driver keeps the setup Symbian left in the chip and
 * only does what /bin/keyscan proved on the phone: poll the event FIFO
 * (0x10), then clear the keypad irqs (0x08 = 3). Polling also avoids the irq
 * line (gpio 42), whose polarity setting in gpio-rapuyama is a guess.
 *
 * FIFO byte: bit 7 = release, bits 4-6 = row, bits 0-3 = col, 0x7f = empty.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#define RK_KBDIC	0x08
#define RK_KBDIC_CLR	0x03
#define RK_FIFO		0x10
#define RK_FIFO_EMPTY	0x7f
#define RK_FIFO_DEPTH	16
#define RK_POLL_MS	20
#define RK_IDLE_MS	40	/* no key for RK_FAST_POLLS polls */
#define RK_FAST_POLLS	25

struct rapuyama_keypad {
	struct i2c_client *client;
	struct input_dev *input;
	unsigned int rows, cols, row_shift;
	unsigned int fast;	/* polls left at RK_POLL_MS */
};

/* returns how many events were in the FIFO */
static int rapuyama_keypad_drain(struct rapuyama_keypad *kp, bool report)
{
	unsigned int row, col, code;
	int i, c;

	for (i = 0; i < RK_FIFO_DEPTH; i++) {
		c = i2c_smbus_read_byte_data(kp->client, RK_FIFO);
		if (c < 0 || (c & 0x7f) == RK_FIFO_EMPTY)
			break;
		row = (c >> 4) & 0x7;
		col = c & 0xf;
		if (!report || row >= kp->rows || col >= kp->cols)
			continue;
		code = MATRIX_SCAN_CODE(row, col, kp->row_shift);
		input_event(kp->input, EV_MSC, MSC_SCAN, code);
		input_report_key(kp->input,
				 ((unsigned short *)kp->input->keycode)[code],
				 !(c & 0x80));
		input_sync(kp->input);
	}
	if (i)
		i2c_smbus_write_byte_data(kp->client, RK_KBDIC, RK_KBDIC_CLR);
	return i;
}

/* every poll is i2c traffic + an irq per byte: slow down while nothing happens */
static void rapuyama_keypad_poll(struct input_dev *input)
{
	struct rapuyama_keypad *kp = input_get_drvdata(input);

	if (rapuyama_keypad_drain(kp, true))
		kp->fast = RK_FAST_POLLS;
	else if (kp->fast)
		kp->fast--;
	input_set_poll_interval(input, kp->fast ? RK_POLL_MS : RK_IDLE_MS);
}

static int rapuyama_keypad_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct rapuyama_keypad *kp;
	int ret;

	ret = i2c_smbus_read_byte_data(client, 0x80);
	if (ret < 0)
		return dev_err_probe(dev, ret, "no answer at 0x%02x\n", client->addr);
	if (ret != 3)
		return dev_err_probe(dev, -ENODEV, "chip id %#x, want 3\n", ret);

	kp = devm_kzalloc(dev, sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;
	kp->client = client;

	ret = matrix_keypad_parse_properties(dev, &kp->rows, &kp->cols);
	if (ret)
		return ret;
	kp->row_shift = get_count_order(kp->cols);

	kp->input = devm_input_allocate_device(dev);
	if (!kp->input)
		return -ENOMEM;
	kp->input->name = "rapuyama-keypad";
	kp->input->id.bustype = BUS_I2C;

	ret = matrix_keypad_build_keymap(NULL, NULL, kp->rows, kp->cols,
					 NULL, kp->input);
	if (ret)
		return dev_err_probe(dev, ret, "bad linux,keymap\n");
	input_set_capability(kp->input, EV_MSC, MSC_SCAN);
	__set_bit(EV_REP, kp->input->evbit);
	/*
	 * udev (and libudev-zero) call a device a keyboard only if it has
	 * all of KEY_ESC..KEY_S, else libinput and weston ignore it. the
	 * qwerty has no digit row (digits are Fn + letters in xkb), so
	 * claim them, they just never fire
	 */
	for (ret = KEY_ESC; ret <= KEY_S; ret++)
		__set_bit(ret, kp->input->keybit);
	input_set_drvdata(kp->input, kp);

	ret = input_setup_polling(kp->input, rapuyama_keypad_poll);
	if (ret)
		return ret;
	input_set_poll_interval(kp->input, RK_POLL_MS);

	/* whatever Symbian left in the FIFO */
	rapuyama_keypad_drain(kp, false);

	ret = input_register_device(kp->input);
	if (ret)
		return ret;
	dev_info(dev, "%ux%u matrix, polled every %d ms (%d ms idle)\n",
		 kp->rows, kp->cols, RK_POLL_MS, RK_IDLE_MS);
	return 0;
}

static const struct of_device_id rapuyama_keypad_of_match[] = {
	{ .compatible = "nokia,rapuyama-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_keypad_of_match);

static struct i2c_driver rapuyama_keypad_driver = {
	.driver = {
		.name = "rapuyama-keypad",
		.of_match_table = rapuyama_keypad_of_match,
	},
	.probe = rapuyama_keypad_probe,
};
module_i2c_driver(rapuyama_keypad_driver);

MODULE_DESCRIPTION("Nokia E6 (RapuYama) TC3589x keypad, polled");
MODULE_LICENSE("GPL");