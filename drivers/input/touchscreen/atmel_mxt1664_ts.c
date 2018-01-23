/*
 * Driver for Atmel MXT1664 Multiple Touch Controller
 *
 * Copyright (C) 2017 EAE Technology
 *
 * Author: Recep Birol Gül
 *		   recepbirolgl@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
//#include <linux/workqueue.h>

#include <linux/bitops.h>
#include <linux/input/mt.h>
#include <linux/amlogic/aml_gpio_consumer.h>
//#include <linux/gpio.h>


#define MAX_SUPPORT_POINTS		5

#define EVENT_VALID_OFFSET	7
#define EVENT_VALID_MASK	(0x1 << EVENT_VALID_OFFSET)
#define EVENT_ID_OFFSET		2
#define EVENT_ID_MASK		(0xf << EVENT_ID_OFFSET)
#define EVENT_IN_RANGE		(0x1 << 1)
#define EVENT_DOWN_UP		(0X1 << 0)

#define MAX_I2C_DATA_LEN	20

#define MXT1664_MAX_X	 	4095
#define MXT1664_MAX_Y	 	4095
#define MXT1664_MAX_TRIES 	 100

static struct workqueue_struct *mxt1664_wq;

struct mxt1664_ts {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	u32 gpio_pendown;
	u32 gpio_reset;
	unsigned pendown;
	int irq;
	int irq_bank;
	struct work_struct	work;
};

/*	// without workqueue
static irqreturn_t mxt1664_ts_read_one_set(struct input_dev *input_dev,
	struct i2c_client *client, u8 *buf)
{
	int id, ret, x, y, z;
	int tries = 0;
	bool down, valid;
	u8 state;

	do {
		ret = i2c_master_recv(client, buf, MAX_I2C_DATA_LEN);
	} while (ret == -EAGAIN && tries++ < MXT1664_MAX_TRIES);

	if (ret < 0)
		return IRQ_HANDLED;

	state = buf[1];
	x = (buf[6] << 8) | buf[5];
	y = (buf[10] << 8) | buf[9];
	z = (buf[14] << 8) | buf[13];

	valid = 1;//state & EVENT_VALID_MASK;
	id = buf[4];//(state & EVENT_ID_MASK) >> EVENT_ID_OFFSET;
	down = buf[3]==1;//state & EVENT_DOWN_UP;

	if (!valid || id > MAX_SUPPORT_POINTS) {
		dev_dbg(&client->dev, "point invalid\n");
		return IRQ_HANDLED;
	}

	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, down);


	if (down) {
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, z);
	}

	input_mt_report_pointer_emulation(input_dev, true);
	input_sync(input_dev);

	return IRQ_HANDLED;
}

*/
static void mxt1664_ts_read_one_set(struct work_struct *work)
{
	int id, ret, x, y, z;
	int tries = 0;
	bool down, valid;
	u8 state;
	u8 buf[MAX_I2C_DATA_LEN];
	struct mxt1664_ts	*ts = container_of(work, struct mxt1664_ts, work);

	do {
		ret = i2c_master_recv(ts->client, buf, MAX_I2C_DATA_LEN);
	} while (ret == -EAGAIN && tries++ < MXT1664_MAX_TRIES);


	state = buf[1];
	x = (buf[6] << 8) | buf[5];
	y = (buf[10] << 8) | buf[9];
	z = (buf[14] << 8) | buf[13];

	valid = 1;//state & EVENT_VALID_MASK;
	id = buf[4];//(state & EVENT_ID_MASK) >> EVENT_ID_OFFSET;
	down = buf[3]==1;//state & EVENT_DOWN_UP;

	if (!valid || id > MAX_SUPPORT_POINTS) 
		dev_dbg(&ts->client->dev, "point invalid\n");
	

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, down);


	if (down) {
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, z);
	}

	input_mt_report_pointer_emulation(ts->input_dev, true);
	input_sync(ts->input_dev);
}

static irqreturn_t mxt1664_hw_irq(int irq, void *handle)
{
	struct mxt1664_ts *ts = handle;

	return	gpio_get_value(ts->gpio_pendown) ? 0 : 1;
}

static irqreturn_t mxt1664_ts_interrupt(int irq, void *dev_id)
{
	struct mxt1664_ts *ts = dev_id;
	struct input_dev *input_dev = ts->input_dev;
	struct i2c_client *client = ts->client;
	u8 buf[MAX_I2C_DATA_LEN];
	//unsigned gpio = (4-1)*32 + 25;//irq_to_gpio(irq); 

	//while (gpio_get_value(gpio) == 0) {
	//mxt1664_ts_read_one_set(input_dev, client, buf);
	queue_work(mxt1664_wq, &ts->work);
	//}

	return IRQ_HANDLED;
}

/* wake up controller with a pulse on the reset gpio.  */
static int mxt1664_wake_up_device(gpio)
{
	/*
	struct device_node *np = client->dev.of_node;
	u32 gpio;
	int ret;

	if (!np)
		return -ENODEV;

	gpio = of_get_named_gpio(np, "gpio-reset", 0);
	if (!gpio_is_valid(gpio))
		return -ENODEV;

	ret = gpio_request(gpio, "mxt1664_wakeup");
	if (ret < 0) {
		dev_err(&client->dev,
			"request gpio failed, cannot wake up controller: %d\n",
			ret);
		return ret;
	}
	*/
	/* wake up controller via an 100ms low period on reset gpio. */
	gpio_direction_output(gpio, 0);
	msleep(100);
	gpio_set_value(gpio, 1);
	msleep(500);

	/* controller should be waken up, return gpio.  */
	gpio_direction_input(gpio);
	gpio_free(gpio);

	return 0;
}

static int mxt1664_ts_enter_app_mode(struct i2c_client *client)
{
	char info[7];
	int loop;
	int ret;
	int original_addr = client->addr;

	for (loop = 1; loop <= 3; loop++) {
		/* read the 7 byte information header from reg 0 */
		info[0] = info[1] = 0x00;
		ret = i2c_master_send(client, info, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to set read address, ret=%d\n", ret);
		} else {
			ret = i2c_master_recv(client, info, 7);
			if (ret < 0) {
				dev_err(&client->dev, "Failed to read information, ret=%d\n", ret);
			} else {
				/* could read the information */
				break;
			}
		}

		/* Check CRC_FAIL bit from bootloader addr 1*/
		client->addr = 0x27;
		ret = i2c_master_recv(client, info, 1);
		if (ret < 0) {
			/* no response from primary address, test secondary */
			dev_err(&client->dev, "No response from bootloader addr 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
			client->addr = 0x28;
			ret = i2c_master_recv(client, info, 1);
			if (ret < 0) {
				/* Neither primary nor secondary answers */
				dev_err(&client->dev, "No response from bootloader addr 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
				dev_err(&client->dev, "No response from bootloaders, aborting\n");
				client->addr = original_addr;
				return ret;
			} else {
				dev_err(&client->dev, "Found bootloader on 0x%02x (%d), CRC=%s\n", client->addr, client->addr, ((info[0]&0xc0)==0x40)?"APP_CRC_FAIL":"");
			}
		} else {
			dev_err(&client->dev, "Found bootloader on 0x%02x (%d), CRC=%s\n", client->addr, client->addr, ((info[0]&0xc0)==0x40)?"APP_CRC_FAIL":"");
		}

		dev_err(&client->dev, "Going into APP mode..\n");
		info[0] = info[1] = 0x01;
		ret = i2c_master_send(client, info, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to send 0x0101 to 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
		}
		msleep(3000);
	}

	if (loop == 4) {
		dev_err(&client->dev, "Failed 3 attempts, aborting\n");
		return -ENODEV;
	}

	dev_info(&client->dev, "Info: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
		info[0], info[1], info[2], info[3], info[4], info[5], info[6]);

	return 0;
}

static int mxt1664_dt_probe(struct i2c_client *client, struct mxt1664_ts *ts)
{
	struct device_node *node = client->dev.of_node;
	int err;

	if(!node) {
		dev_err(&client->dev,
			"mxt1664_dt_probe: Device do not have associated DT data\n");
		return -EINVAL;
	}

	ts->gpio_pendown = 98;//of_get_named_gpio(node, "gpio-pendown", 0);
	ts->gpio_reset	 = 99;//of_get_named_gpio(node, "gpio-reset", 0);

	err = amlogic_gpio_request_one(ts->gpio_pendown, GPIOF_IN, "ts-pendown");

	if (err) {
		dev_err(&client->dev,
			"mxt1664_ts: failed to request/setup pendown GPIO%d: %d\n",
			ts->gpio_pendown, err);
		return -EINVAL;		
	}

	err = amlogic_gpio_request_one(ts->gpio_reset, GPIOF_OUT_INIT_LOW, "ts-reset");
		
	if (err) {
		dev_err(&client->dev,
			"mxt1664_ts: failed to request/setup reset GPIO%d: %d\n",
			ts->gpio_reset, err);
		return -EINVAL;
	}


	ts->irq_bank = meson_fix_irqbank(ts->irq_bank);
	if (ts->irq_bank < 0) {
		dev_err(&client->dev,
			"mxt1664_ts: Could not find irq bank!\n");
		return -EINVAL;
	}

	err = amlogic_gpio_to_irq(ts->gpio_pendown, "ts-pendown",
		AML_GPIO_IRQ(ts->irq_bank, FILTER_NUM7, GPIO_IRQ_FALLING));

	if (err) {
		dev_err(&client->dev,
			"mxt1664_ts: AML_GPIO_IRQ setup fail!\n");
		return -EINVAL;
	}
		/* Amlogic gpio based irq setup */
	ts->irq = INT_GPIO_0 + ts->irq_bank;	
/*
	if(gpio_request(ts->gpio_pendown, "ts-pendown"))
		dev_err(&client->dev,
			"mxt1664_dt_probe: pendown gpio request fail (%d)!\n", ts->gpio_pendown);
	else {
		dev_info(&client->dev,
			"mxt1664_dt_probe: pendown gpio request success (%d)!\n", ts->gpio_pendown);
		gpio_direction_input(ts->gpio_pendown);
	}

	if(gpio_request(ts->gpio_reset, "ts-reset"))
		dev_err(&client->dev,
			"mxt1664_dt_probe: reset gpio request fail (%d)\n", ts->gpio_reset);
	else {
		dev_info(&client->dev,
			"mxt1664_dt_probe: reset gpio request success (%d)!\n", ts->gpio_pendown);
		// wake up fonksiyonu çağırılacak
	}
	
	ts->irq = 96;//gpio_to_irq(ts->gpio_pendown);
	if (ts->irq < 0) {
		dev_err(&client->dev,
			"mxt1664_dt_probe: gpio_to_irq() fail (%d)\n", ts->irq);
		return -EINVAL;
	}
*/
	/* platform data info display */
	dev_info(&client->dev, "mxt1664_ts: gpio pendown (%d)\n",	ts->gpio_pendown);
	dev_info(&client->dev, "mxt1664_ts: gpio reset (%d)\n",	ts->gpio_reset);
	dev_info(&client->dev, "mxt1664_ts: gpio irq (%d)\n",	ts->irq);
	dev_info(&client->dev, "mxt1664_ts: client irq (%d)\n", client->irq);		
	
	dev_info(&client->dev,
			"mxt1664_dt_probe: probe ok! ts->irq: (%d)\n", ts->irq);

	return 0;
}

static int mxt1664_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct mxt1664_ts *ts;
	struct input_dev *input_dev;
	int err = 0;

	dev_info(&client->dev, "mxt1664_ts_probe()\n");

	ts = devm_kzalloc(&client->dev, sizeof(struct mxt1664_ts), GFP_KERNEL);
	input_dev = devm_input_allocate_device(&client->dev);
	
	if (!ts || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	if (mxt1664_dt_probe(client, ts) != 0)
		return -EIO;


	ts->client = client;
	ts->input_dev = input_dev;

	mxt1664_wake_up_device(ts->gpio_reset);

	INIT_WORK(&ts->work, mxt1664_ts_read_one_set); // init work_struct

	//msleep(600);
	
	err = mxt1664_ts_enter_app_mode(client);
	if (err) {
		dev_err(&client->dev, "Failed to put controller in app mode\n");
		//return err;
	}

	
	//mxt1664_wq = create_singlethread_workqueue("mxt1664_wq");

	input_dev->name = "Atmel MXT1664 Touch Screen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, MXT1664_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MXT1664_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, MXT1664_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, MXT1664_MAX_Y, 0, 0);
	input_mt_init_slots(input_dev, MAX_SUPPORT_POINTS, 0);

	input_set_drvdata(input_dev, ts);

	err = request_irq(ts->irq,
					  mxt1664_ts_interrupt,
					  IRQF_DISABLED ,
					  "mxt1664_ts", ts);

	if (err < 0) {
		dev_err(&client->dev, 
			"Failed to register interrupt err: %d\n", err);
		return err;
	}

	err = input_register_device(ts->input_dev);
	if (err)
		return err;

	i2c_set_clientdata(client, ts);

	dev_info(&client->dev, "probe ok! registered with irq (%d)\n", ts->irq);

	return 0;
}


static int __maybe_unused mxt1664_ts_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused mxt1664_ts_resume(struct device *dev)
{
	return 0;
}

static const struct of_device_id mxt1664_ts_dt_ids[] = {
	{ .compatible = "atmel,mxt1664_ts", .data = (void *)0},
	{}
};

static const struct i2c_device_id mxt1664_ts_id[] = {
	{ "mxt1664_ts", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, mxt1664_ts_id);

MODULE_DEVICE_TABLE(of, mxt1664_ts_dt_ids);

static SIMPLE_DEV_PM_OPS(mxt1664_ts_pm_ops, mxt1664_ts_suspend, mxt1664_ts_resume);

static struct i2c_driver mxt1664_ts_driver = {
	.driver = {
		.name	= "mxt1664_ts",
		.owner	= THIS_MODULE,
		.pm	= &mxt1664_ts_pm_ops,
		.of_match_table	= of_match_ptr(mxt1664_ts_dt_ids),
	},
	.id_table	= mxt1664_ts_id,
	.probe		= mxt1664_ts_probe,
};



/*******************************
* driver init ve exit işlemleri
*******************************/


static int __init mxt1664_ts_init(void)
{
	int ret;
	
	mxt1664_wq = create_workqueue("mxt1664_wq");

	if (!mxt1664_wq) {
		printk(KERN_DEBUG "create workqueue failed\n");
		return -ENOMEM;
	}

	printk(KERN_DEBUG "work queue created");
	
	ret=i2c_add_driver(&mxt1664_ts_driver);
	
	if (ret) {
		printk(KERN_DEBUG "adding mxt1664 driver failed.");
		return ret;
	} 

	printk(KERN_DEBUG "adding mxt1664 driver");
	return ret;
}

static void __exit mxt1664_ts_exit(void)
{
	printk(KERN_DEBUG "mx1664 TouchScreen driver exited.\n");
	i2c_del_driver(&mxt1664_ts_driver);

	if (mxt1664_wq)
		destroy_workqueue(mxt1664_wq); // release work queue
}

module_init(mxt1664_ts_init);
module_exit(mxt1664_ts_exit);
//module_i2c_driver(mxt1664_ts_driver);	

MODULE_AUTHOR("EAE Technology");
MODULE_DESCRIPTION("Touchscreen driver for Atmel MXT1664 touch controller");
MODULE_LICENSE("GPL");
