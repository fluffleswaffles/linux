// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E6 touchscreen: Atmel maXTouch on i2c 0x4a (atmel_qt.dll). Polled and
 * keeps what Symbian left in the chip, the only write wakes T7 if it sleeps
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/unaligned.h>

#define MXT_OBJ_T5	5	/* message processor */
#define MXT_OBJ_T7	7	/* power config */
#define MXT_OBJ_T9	9	/* multiple touch */
#define MXT_OBJ_T100	100	/* newer multiple touch, not handled yet */
#define MXT_MAX_OBJ	64
#define MXT_MAX_MSG	16
#define MXT_POLL_MS	10
#define MXT_DRAIN	16	/* messages per poll at most */
#define MXT_SLOTS	10

/* T9 message status */
#define MXT_T9_DETECT	BIT(7)
#define MXT_T9_SUPPRESS	BIT(1)

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "log raw T9 touches");

struct rapuyama_mxt {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	u16 t5_addr, t7_addr, t9_addr;
	u8 t5_size, t9_min_id, t9_max_id;
	u16 xmax, ymax;
};

/* the chip takes a little endian 16-bit register address first */
static int mxt_read(struct i2c_client *c, u16 reg, void *buf, u16 len)
{
	u8 a[2];
	struct i2c_msg m[2] = {
		{ .addr = c->addr, .len = 2, .buf = a },
		{ .addr = c->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret;

	put_unaligned_le16(reg, a);
	ret = i2c_transfer(c->adapter, m, 2);
	return ret == 2 ? 0 : ret < 0 ? ret : -EIO;
}

static int mxt_write(struct i2c_client *c, u16 reg, const void *buf, u16 len)
{
	u8 b[2 + 8];
	int ret;

	if (len > 8)
		return -EINVAL;
	put_unaligned_le16(reg, b);
	memcpy(b + 2, buf, len);
	ret = i2c_master_send(c, b, len + 2);
	return ret == len + 2 ? 0 : ret < 0 ? ret : -EIO;
}

static void mxt_t9(struct rapuyama_mxt *ts, const u8 *m)
{
	int slot = m[0] - ts->t9_min_id;
	u8 st = m[1];
	unsigned int x, y;

	/* 12 bit, 10 bit for a range below 1024 (as atmel_mxt_ts) */
	x = (m[2] << 4) | (m[4] >> 4);
	y = (m[3] << 4) | (m[4] & 0xf);
	if (ts->xmax < 1024)
		x >>= 2;
	if (ts->ymax < 1024)
		y >>= 2;
	if (debug)
		dev_info(&ts->client->dev, "touch %d status %#x x %u y %u area %u\n",
			 slot, st, x, y, m[5]);
	if (slot >= MXT_SLOTS)
		return;
	input_mt_slot(ts->input, slot);
	if ((st & MXT_T9_DETECT) && !(st & MXT_T9_SUPPRESS)) {
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, m[5]);
	} else {
		input_mt_report_slot_inactive(ts->input);
	}
}

static void rapuyama_mxt_poll(struct input_dev *input)
{
	struct rapuyama_mxt *ts = input_get_drvdata(input);
	u8 m[MXT_MAX_MSG];
	bool any = false;
	int i;

	for (i = 0; i < MXT_DRAIN; i++) {
		if (mxt_read(ts->client, ts->t5_addr, m, ts->t5_size))
			break;
		if (m[0] == 0xff)
			break;
		if (m[0] >= ts->t9_min_id && m[0] <= ts->t9_max_id) {
			mxt_t9(ts, m);
			any = true;
		} else if (debug) {
			dev_info(&ts->client->dev, "message %*ph\n", ts->t5_size, m);
		}
	}
	if (any) {
		input_mt_sync_frame(input);
		input_sync(input);
	}
}

/* walk the object table, note T5 / T7 / T9 and the T9 report ids */
static int mxt_objects(struct rapuyama_mxt *ts, u8 num)
{
	struct device *dev = &ts->client->dev;
	u8 o[6], id = 1;
	int i, ret;

	for (i = 0; i < num && i < MXT_MAX_OBJ; i++) {
		u16 start;
		u8 size, inst, ids;

		ret = mxt_read(ts->client, 7 + 6 * i, o, 6);
		if (ret)
			return ret;
		start = get_unaligned_le16(o + 1);
		size = o[3] + 1;
		inst = o[4] + 1;
		ids = o[5] * inst;
		dev_info(dev, "T%u at %#x, %u bytes x %u, report ids %u-%u\n",
			 o[0], start, size, inst, ids ? id : 0, ids ? id + ids - 1 : 0);
		switch (o[0]) {
		case MXT_OBJ_T5:
			ts->t5_addr = start;
			ts->t5_size = min_t(u8, size, MXT_MAX_MSG);
			break;
		case MXT_OBJ_T7:
			ts->t7_addr = start;
			break;
		case MXT_OBJ_T9:
			ts->t9_addr = start;
			ts->t9_min_id = id;
			ts->t9_max_id = id + ids - 1;
			break;
		case MXT_OBJ_T100:
			dev_warn(dev, "T100 found: touches come in T100 messages, not decoded yet\n");
			break;
		}
		id += ids;
	}
	return ts->t5_addr && ts->t9_addr ? 0 : -ENODEV;
}

static int rapuyama_mxt_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct rapuyama_mxt *ts;
	struct input_dev *input;
	u8 info[7], t9[22], t7[2];
	int ret;

	ret = mxt_read(client, 0, info, sizeof(info));
	if (ret)
		return dev_err_probe(dev, ret, "no answer at 0x%02x\n", client->addr);
	dev_info(dev, "maXTouch family %#x variant %#x version %u.%u build %#x, matrix %ux%u, %u objects\n",
		 info[0], info[1], info[2] >> 4, info[2] & 0xf, info[3],
		 info[4], info[5], info[6]);

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->client = client;
	ret = mxt_objects(ts, info[6]);
	if (ret)
		return dev_err_probe(dev, ret, "no T5 / T9\n");

	/* T9 config: orient at 9, x range 18-19, y range 20-21 */
	ret = mxt_read(client, ts->t9_addr, t9, sizeof(t9));
	if (ret)
		return ret;
	ts->xmax = get_unaligned_le16(t9 + 18);
	ts->ymax = get_unaligned_le16(t9 + 20);
	if (t9[9] & 1)		/* the chip swaps x / y itself */
		swap(ts->xmax, ts->ymax);
	if (!ts->xmax)
		ts->xmax = 1023;
	if (!ts->ymax)
		ts->ymax = 1023;
	dev_info(dev, "T9 ctrl %#x orient %#x range %u x %u\n", t9[0], t9[9],
		 ts->xmax, ts->ymax);

	if (ts->t7_addr && !mxt_read(client, ts->t7_addr, t7, 2)) {
		dev_info(dev, "T7 idle %u active %u\n", t7[0], t7[1]);
		if (!t7[0] && !t7[1]) {
			static const u8 run[2] = { 32, 16 };

			ret = mxt_write(client, ts->t7_addr, run, 2);
			dev_info(dev, "T7 was asleep, woken: %d\n", ret);
		}
	}

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	ts->input = input;
	input->name = "E6 touchscreen";
	input->id.bustype = BUS_I2C;
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->xmax, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->ymax, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);
	ret = input_mt_init_slots(input, MXT_SLOTS, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;
	input_set_drvdata(input, ts);
	ret = input_setup_polling(input, rapuyama_mxt_poll);
	if (ret)
		return ret;
	input_set_poll_interval(input, MXT_POLL_MS);
	return input_register_device(input);
}

static const struct of_device_id rapuyama_mxt_of_match[] = {
	{ .compatible = "nokia,rapuyama-mxt" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_mxt_of_match);

static struct i2c_driver rapuyama_mxt_driver = {
	.driver = {
		.name = "rapuyama-mxt",
		.of_match_table = rapuyama_mxt_of_match,
	},
	.probe = rapuyama_mxt_probe,
};
module_i2c_driver(rapuyama_mxt_driver);

MODULE_DESCRIPTION("Nokia E6 (RapuYama) maXTouch touchscreen, polled");
MODULE_LICENSE("GPL");