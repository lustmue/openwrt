// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/serdev.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>
#include <linux/leds.h>
#include <linux/hwmon.h>

#define WT69P3_FRAME_START	0xfa
#define WT69P3_FRAME_END	0xfb
#define WT69P3_CMD_LEN	7
#define WT69P3_MAX_RX		16

#define WT69P3_LED_MODE_OFF	0x00
#define WT69P3_LED_MODE_ON	0x01
#define WT69P3_LED_MODE_BLINK	0x02

struct wt69p3 {
	struct serdev_device *serdev;
	struct rtc_device *rtc;
	struct mutex lock;
	struct completion rx_done;
	u8 rx_buf[WT69P3_MAX_RX];
	size_t rx_len;
};

static size_t wt69p3_receive_buf(struct serdev_device *serdev,
				   const u8 *buf, size_t count)
{
	struct wt69p3 *mcu = serdev_device_get_drvdata(serdev);
	size_t i;

	for (i = 0; i < count; i++) {
		if (mcu->rx_len == 0 && buf[i] != WT69P3_FRAME_START)
			continue;

		if (mcu->rx_len < sizeof(mcu->rx_buf))
			mcu->rx_buf[mcu->rx_len++] = buf[i];

		if (buf[i] == WT69P3_FRAME_END && mcu->rx_len >= WT69P3_CMD_LEN) {
			complete(&mcu->rx_done);
			break;
		}
	}

	return count;
}

static void wt69p3_serdev_close(void *data)
{
	serdev_device_close(data);
}

static const struct serdev_device_ops wt69p3_serdev_ops = {
	.receive_buf = wt69p3_receive_buf,
};

static int wt69p3_send_cmd(struct wt69p3 *mcu, const u8 *cmd, size_t len)
{
	int ret;

	mutex_lock(&mcu->lock);
	ret = serdev_device_write_buf(mcu->serdev, cmd, len);
	serdev_device_wait_until_sent(mcu->serdev, msecs_to_jiffies(100));
	mutex_unlock(&mcu->lock);

	return ret < 0 ? ret : 0;
}

static int wt69p3_query_cmd(struct wt69p3 *mcu, const u8 *cmd,
			      size_t len, u8 *reply, size_t reply_max)
{
	int ret;
	unsigned long timeout;

	mutex_lock(&mcu->lock);
	mcu->rx_len = 0;
	reinit_completion(&mcu->rx_done);

	ret = serdev_device_write_buf(mcu->serdev, cmd, len);
	if (ret < 0) {
		mutex_unlock(&mcu->lock);
		return ret;
	}
	serdev_device_wait_until_sent(mcu->serdev, msecs_to_jiffies(100));

	timeout = wait_for_completion_timeout(&mcu->rx_done, msecs_to_jiffies(300));
	if (!timeout) {
		mutex_unlock(&mcu->lock);
		return -ETIMEDOUT;
	}

	ret = min(mcu->rx_len, reply_max);
	memcpy(reply, mcu->rx_buf, ret);
	mutex_unlock(&mcu->lock);

	return ret;
}

static int wt69p3_set_wol(struct wt69p3 *mcu, bool enable)
{
	u8 cmd[WT69P3_CMD_LEN] = {
		WT69P3_FRAME_START,
		0x03,
		0x0a,
		enable ? 0x01 : 0x00,
		0x00,
		0x00,
		WT69P3_FRAME_END
	};

	return wt69p3_send_cmd(mcu, cmd, sizeof(cmd));
}

static umode_t wt69p3_hwmon_is_visible(const void *drvdata,
		enum hwmon_sensor_types type,
		u32 attr, int channel)
{
	switch (type) {
	case hwmon_temp:
		if (attr == hwmon_temp_input)
			return 0444;
		break;
	default:
		break;
	}

	return 0;
}

static int wt69p3_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct wt69p3 *mcu = dev_get_drvdata(dev);
	static const u8 rtc_read_cmd[WT69P3_CMD_LEN] = {
		WT69P3_FRAME_START,
		0x01,
		0x08,
		0x01,
		0x01,
		0x00,
		WT69P3_FRAME_END
	};
	u8 reply[WT69P3_MAX_RX];
	int ret;

	ret = wt69p3_query_cmd(mcu, rtc_read_cmd, sizeof(rtc_read_cmd),
				 reply, sizeof(reply));
	if (ret < 13)
		return ret < 0 ? ret : -EIO;

	if (reply[0] != WT69P3_FRAME_START || 
			reply[1] != 0x01 ||
			reply[2] != 0x08 ||
			reply[12] != WT69P3_FRAME_END)
		return -EIO;

	tm->tm_sec  = bcd2bin(reply[5]);
	tm->tm_min  = bcd2bin(reply[6]);
	tm->tm_hour = bcd2bin(reply[7]);
	tm->tm_wday = bcd2bin(reply[8]) % 7;
	tm->tm_mday = bcd2bin(reply[9]);
	tm->tm_mon  = bcd2bin(reply[10]) - 1;
	tm->tm_year = bcd2bin(reply[11]) + 100;

	return rtc_valid_tm(tm);
}

static int wt69p3_rtc_write_reg(struct wt69p3 *mcu, u8 reg, u8 bcd_val)
{
	u8 cmd[WT69P3_CMD_LEN] = {
		WT69P3_FRAME_START,
		0x01,
		reg,
		0x01,
		0x01,
		bcd_val,
		WT69P3_FRAME_END
	};
	u8 reply[WT69P3_MAX_RX];
	int ret;

	ret = wt69p3_query_cmd(mcu, cmd, sizeof(cmd), reply, sizeof(reply));
	if (ret < 0)
		return ret;

	if (ret >= 2 && reply[1] == 0x30)
		return -EIO;

	return 0;
}

static int wt69p3_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct wt69p3 *mcu = dev_get_drvdata(dev);
	const struct {
		u8 reg;
		u8 val;
	} fields[] = {
		{ 0x01, bin2bcd(tm->tm_sec) },
		{ 0x02, bin2bcd(tm->tm_min) },
		{ 0x03, bin2bcd(tm->tm_hour) },
		{ 0x04, bin2bcd(tm->tm_wday) },
		{ 0x05, bin2bcd(tm->tm_mday) },
		{ 0x06, bin2bcd(tm->tm_mon + 1) },
		{ 0x07, bin2bcd(tm->tm_year - 100) },
	};
	int i, ret;

	if (tm->tm_year < 100 || tm->tm_year > 199)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		ret = wt69p3_rtc_write_reg(mcu, fields[i].reg, fields[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct rtc_class_ops wt69p3_rtc_ops = {
	.read_time = wt69p3_rtc_read_time,
	.set_time = wt69p3_rtc_set_time,
};

static int wt69p3_read_temp(struct wt69p3 *mcu, long *val)
{
	static const u8 temp_cmd[WT69P3_CMD_LEN] = {
		WT69P3_FRAME_START,
		0x03,
		0x08,
		0x00,
		0x00,
		0x00,
		WT69P3_FRAME_END
	};
	u8 reply[WT69P3_MAX_RX];
	int ret;

	ret = wt69p3_query_cmd(mcu, temp_cmd, sizeof(temp_cmd),
					 reply, sizeof(reply));
	if (ret < WT69P3_CMD_LEN)
		return ret < 0 ? ret : -EIO;

	*val = (((long)reply[5] - 32) * 5000) / 9;

	return 0;
}

static int wt69p3_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long *val)
{
	struct wt69p3 *mcu = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		if (attr != hwmon_temp_input)
			return -EOPNOTSUPP;
		return wt69p3_read_temp(mcu, val);

	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_channel_info * const wt69p3_hwmon_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_ops wt69p3_hwmon_ops = {
	.is_visible = wt69p3_hwmon_is_visible,
	.read = wt69p3_hwmon_read,
};

static const struct hwmon_chip_info wt69p3_chip_info = {
	.ops = &wt69p3_hwmon_ops,
	.info = wt69p3_hwmon_info,
};

static int wt69p3_power_off_prepare(struct sys_off_data *data)
{
	struct wt69p3 *mcu = data->cb_data;
	static const u8 cmd[WT69P3_CMD_LEN] = {
		WT69P3_FRAME_START,
		0x03,
		0x03,
		0x01,
		0x01,
		0x14,
		WT69P3_FRAME_END
	};
	int ret;

	wt69p3_set_wol(mcu, device_may_wakeup(&mcu->serdev->dev));

	ret = wt69p3_send_cmd(mcu, cmd, sizeof(cmd));
	if (ret) {
		return notifier_from_errno(ret);
	}

	return NOTIFY_DONE;
}

struct wt69p3_led {
	struct led_classdev cdev;
	struct wt69p3 *mcu;
	u8 reg;
};

static int wt69p3_led_write_mode(struct wt69p3_led *led, u8 mode)
{
	u8 cmd[WT69P3_CMD_LEN] = {
		WT69P3_FRAME_START,
		0x03,
		0x06,
		mode,
		0x00,
		0x01,
		WT69P3_FRAME_END
	};

	return wt69p3_send_cmd(led->mcu, cmd, sizeof(cmd));
}

static int wt69p3_led_set(struct led_classdev *led_cdev,
			  enum led_brightness brightness)
{
	struct wt69p3_led *led = container_of(led_cdev, struct wt69p3_led, cdev);

	return wt69p3_led_write_mode(led, brightness ?
			WT69P3_LED_MODE_ON : WT69P3_LED_MODE_OFF);
}

static int wt69p3_led_blink_set(struct led_classdev *led_cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	struct wt69p3_led *led = container_of(led_cdev, struct wt69p3_led, cdev);

	if (*delay_on == 0 && *delay_off == 0) {
		*delay_on = 500;
		*delay_off = 500;
	}

	if (*delay_on == 500 && *delay_off == 500)
		return wt69p3_led_write_mode(led, WT69P3_LED_MODE_BLINK);

	return -EINVAL;
}

static int wt69p3_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct wt69p3 *mcu;
	struct device_node *hwmon_np, *rtc_np;
	struct device *hwmon_dev;
	struct fwnode_handle *leds_fwnode, *child;
	static const u8 cmd[WT69P3_CMD_LEN] = {
		WT69P3_FRAME_START,
		0x03,
		0x01,
		0x00,
		0x00,
		0x00,
		WT69P3_FRAME_END
	};
	int ret;

	mcu = devm_kzalloc(&serdev->dev, sizeof(*mcu), GFP_KERNEL);
	if (!mcu)
		return -ENOMEM;

	mcu->serdev = serdev;
	mutex_init(&mcu->lock);
	init_completion(&mcu->rx_done);
	serdev_device_set_drvdata(serdev, mcu);
	serdev_device_set_client_ops(serdev, &wt69p3_serdev_ops);
	ret = serdev_device_open(serdev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&serdev->dev, wt69p3_serdev_close, serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(serdev, 115200);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

	/* Signal "OS load done" */
	wt69p3_send_cmd(mcu, cmd, sizeof(cmd));

	device_init_wakeup(dev, device_property_read_bool(dev, "wakeup-source"));

	/* Register HWMON temperature sensor */
	hwmon_np = of_get_child_by_name(dev->of_node, "hwmon");
	if (hwmon_np) {
		struct device_node *orig_np = dev->of_node;

		dev->of_node = hwmon_np;
		hwmon_dev = devm_hwmon_device_register_with_info(dev, "wt69p3", mcu,
							 &wt69p3_chip_info, NULL);
		dev->of_node = orig_np;
		of_node_put(hwmon_np);

		if (IS_ERR(hwmon_dev))
			return PTR_ERR(hwmon_dev);
	}

	/* Register RTC device */
	rtc_np = of_get_child_by_name(dev->of_node, "rtc");
	if (rtc_np) {
		mcu->rtc = devm_rtc_allocate_device(dev);
		if (IS_ERR(mcu->rtc)) {
			of_node_put(rtc_np);
			return PTR_ERR(mcu->rtc);
		}

		mcu->rtc->dev.of_node = rtc_np;
		mcu->rtc->ops = &wt69p3_rtc_ops;

		ret = devm_rtc_register_device(mcu->rtc);
		of_node_put(rtc_np);
		if (ret)
			return ret;
	}

	/* Register LEDs */
	leds_fwnode = device_get_named_child_node(dev, "leds");
	fwnode_for_each_child_node(leds_fwnode, child) {
		struct led_init_data init_data = { .fwnode = child };
		struct wt69p3_led *led;
		enum led_default_state state = led_init_default_state_get(child);
		u32 reg;

		if (fwnode_property_read_u32(child, "reg", &reg))
			continue;

		led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
		if (!led) {
			fwnode_handle_put(child);
			fwnode_handle_put(leds_fwnode);
			return -ENOMEM;
		}
		led->mcu = mcu;
		led->reg = reg;
		led->cdev.max_brightness = 1;
		led->cdev.brightness_set_blocking = wt69p3_led_set;
		led->cdev.blink_set = wt69p3_led_blink_set;
		switch (state) {
		case LEDS_DEFSTATE_ON:
			led->cdev.brightness = 1;
			break;
		case LEDS_DEFSTATE_KEEP:
			break;
		case LEDS_DEFSTATE_OFF:
		default:
			led->cdev.brightness = LED_OFF;
		}
		ret = devm_led_classdev_register_ext(dev, &led->cdev,
							&init_data);
		if (ret) {
			fwnode_handle_put(child);
			fwnode_handle_put(leds_fwnode);
			return ret;
		}
		if (state != LEDS_DEFSTATE_KEEP)
			wt69p3_led_set(&led->cdev, led->cdev.brightness);
	}
	fwnode_handle_put(leds_fwnode);

	ret = devm_register_sys_off_handler(&serdev->dev,
			SYS_OFF_MODE_POWER_OFF_PREPARE,
			SYS_OFF_PRIO_DEFAULT,
			wt69p3_power_off_prepare,
			mcu);
	if (ret)
		return dev_err_probe(&serdev->dev, ret,
				"Failed to register power-off handler\n");

	return 0;
}

static const struct of_device_id wt69p3_of_match[] = {
	{ .compatible = "weltrend,wt69p3" },
	{ }
};
MODULE_DEVICE_TABLE(of, wt69p3_of_match);

static struct serdev_device_driver wt69p3_driver = {
	.probe = wt69p3_probe,
	.driver = {
		.name = "wt69p3",
		.of_match_table = wt69p3_of_match,
	},
};
module_serdev_device_driver(wt69p3_driver);

MODULE_AUTHOR("Lutz Müller <github@topfrose.de>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Weltrend Control MCU serdev LED, hwmon and RTC driver");
