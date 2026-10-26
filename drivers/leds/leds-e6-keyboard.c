// SPDX-License-Identifier: GPL-2.0-only
/* keep the lp5523 setup left by the E6 firmware, only change keypad PWM */
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>

static const u8 channels[]={0, 1, 2, 4, 5};

struct e6_keyboard{
	struct i2c_client *client;
	struct led_classdev led;
	struct mutex lock;
	u8 pwm[ARRAY_SIZE(channels)];
};

static int e6_keyboard_set(struct led_classdev *led, enum led_brightness value){
	struct e6_keyboard *kbd=container_of(led, struct e6_keyboard, led);
	int ret=0, i;

	mutex_lock(&kbd->lock);
	for(i=0; i<ARRAY_SIZE(channels); i++){
		ret=i2c_smbus_write_byte_data(kbd->client, 0x16+channels[i], value);
		if(ret){
			/* restore the whole group after a partial transfer */
			for(int j=0; j<=i; j++){
				i2c_smbus_write_byte_data(kbd->client, 0x16+channels[j], kbd->pwm[j]);
			}
			break;
		}
	}
	if(!ret){memset(kbd->pwm, value, sizeof(kbd->pwm));}
	mutex_unlock(&kbd->lock);
	return ret;
}

static int e6_keyboard_probe(struct i2c_client *client){
	struct e6_keyboard *kbd;
	int ret;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)){return -EOPNOTSUPP;}
	/* adopting running engines would let them overwrite our PWM values */
	ret=i2c_smbus_read_byte_data(client, 0x01);
	if(ret<0){return ret;}
	if(ret){return dev_err_probe(&client->dev, -EBUSY, "lp5523 engines still running\n");}
	kbd=devm_kzalloc(&client->dev, sizeof(*kbd), GFP_KERNEL);
	if(!kbd){return -ENOMEM;}
	kbd->client=client;
	mutex_init(&kbd->lock);
	for(int i=0; i<ARRAY_SIZE(channels); i++){
		ret=i2c_smbus_read_byte_data(client, 0x06+channels[i]);
		if(ret<0){return ret;}
		if(ret&0xc0){return dev_err_probe(&client->dev, -EBUSY, "keypad channel uses a master fader\n");}
		ret=i2c_smbus_read_byte_data(client, 0x16+channels[i]);
		if(ret<0){return ret;}
		kbd->pwm[i]=ret;
	}
	kbd->led.name="e6:white:kbd_backlight";
	kbd->led.max_brightness=255;
	kbd->led.brightness=kbd->pwm[0];
	kbd->led.brightness_set_blocking=e6_keyboard_set;
	i2c_set_clientdata(client, kbd);
	return devm_led_classdev_register(&client->dev, &kbd->led);
}

static const struct of_device_id e6_keyboard_of_match[]={
	{.compatible="nokia,e6-keyboard-backlight"},
	{}
};
MODULE_DEVICE_TABLE(of, e6_keyboard_of_match);

static const struct i2c_device_id e6_keyboard_ids[]={
	{"e6-kbd-backlight", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, e6_keyboard_ids);

static struct i2c_driver e6_keyboard_driver={
	.driver={.name="e6-keyboard-backlight", .of_match_table=e6_keyboard_of_match},
	.probe=e6_keyboard_probe,
	.id_table=e6_keyboard_ids,
};
module_i2c_driver(e6_keyboard_driver);

MODULE_DESCRIPTION("Nokia E6 keypad backlight on the firmware-configured LP5523");
MODULE_LICENSE("GPL");