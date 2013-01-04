/*
 *  RaspberryPi SC1602 LCD driver
 *
 *  Copyright (C) 2013 Tadaki SAKAI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>

#define BCM2708_PERIPHERAL_BASE	0x20000000
#define GPIO_BASE		(BCM2708_PERIPHERAL_BASE + 0x200000) /* GPIO base */
#define RASP_GPIO_NUM_MAX	26 /* number of gpio lines */

#define GPIO_FSEL_INPUT		0
#define GPIO_FSEL_OUTPUT	1

/* GPIO PIN Number define (Default) */
#define SC1602_GPIO_REGISTER_SELECT	10	/* PIN19 */
#define SC1602_GPIO_ENABLE_SIGNAL	9	/* PIN21 */
/* #define SC1602_GPIO_READ_WRITE	GND	*//* PIN25 (write only) */
#define SC1602_GPIO_DATA_BIT4		25	/* PIN22 (gpio_bank = 0) */
#define SC1602_GPIO_DATA_BIT5		11	/* PIN23 (gpio_bank = 0) */
#define SC1602_GPIO_DATA_BIT6		8	/* PIN24 (gpio_bank = 0) */
#define SC1602_GPIO_DATA_BIT7		7	/* PIN26 (gpio_bank = 0) */

static int sc1602_gpio_rs = SC1602_GPIO_REGISTER_SELECT;
static int sc1602_gpio_e = SC1602_GPIO_ENABLE_SIGNAL;
static int sc1602_gpio_db4 = SC1602_GPIO_DATA_BIT4;
static int sc1602_gpio_db5 = SC1602_GPIO_DATA_BIT5;
static int sc1602_gpio_db6 = SC1602_GPIO_DATA_BIT6;
static int sc1602_gpio_db7 = SC1602_GPIO_DATA_BIT7;

module_param(sc1602_gpio_rs, int, S_IRUGO | S_IWUSR);
module_param(sc1602_gpio_e, int, S_IRUGO | S_IWUSR);
module_param(sc1602_gpio_db4, int, S_IRUGO | S_IWUSR);
module_param(sc1602_gpio_db5, int, S_IRUGO | S_IWUSR);
module_param(sc1602_gpio_db6, int, S_IRUGO | S_IWUSR);
module_param(sc1602_gpio_db7, int, S_IRUGO | S_IWUSR);

#define SC1602_REGISTER_SELECT_COMMAND	0
#define SC1602_REGISTER_SELECT_DATA	1

#define SC1602_DATA_DISABLE		0
#define SC1602_DATA_ENABLE		1

/* SC1602 Instruction code */
#define SC1602_INSTRUCTION_CLEAR_DISPLAY	(1 << 0)
#define SC1602_INSTRUCTION_CURSOR_AT_HOME	(1 << 1)
#define		SC1602_INSTRUCTION_ENTRY_MODE	(1 << 2)
#define	SC1602_INSTRUCTION_ENTRY_INCREMEMNT_MODE	\
	(SC1602_INSTRUCTION_ENTRY_MODE | (1 << 1))
#define	SC1602_INSTRUCTION_ENTRY_DECREMENT_MODE	\
	(SC1602_INSTRUCTION_ENTRY_MODE | (0 << 1))
#define 	SC1602_INSTRUCTION_DISPLAY_CONTROL	(1 << 3)
#define 	SC1602_INSTRUCTION_DISPLAY_ON		(1 << 2)
#define 	SC1602_INSTRUCTION_DISPLAY_OFF		(0 << 2)
#define 	SC1602_INSTRUCTION_DISPLAY_CURSOL_ON	(1 << 1)
#define 	SC1602_INSTRUCTION_DISPLAY_CURSOL_OFF	(0 << 1)
#define 	SC1602_INSTRUCTION_DISPLAY_BLINK_ON	(1 << 0)
#define 	SC1602_INSTRUCTION_DISPLAY_BLINK_OFF	(0 << 0)
#define	SC1602_INSTRUCTION_DISPLAY_CONTROL_ON	\
	(SC1602_INSTRUCTION_DISPLAY_CONTROL |	\
	 SC1602_INSTRUCTION_DISPLAY_ON |	\
	 SC1602_INSTRUCTION_DISPLAY_CURSOL_ON |	\
	 SC1602_INSTRUCTION_DISPLAY_BLINK_OFF)
#define	SC1602_INSTRUCTION_DISPLAY_CONTROL_OFF	\
	(SC1602_INSTRUCTION_DISPLAY_CONTROL |	\
	 SC1602_INSTRUCTION_DISPLAY_OFF)
#define		SC1602_INSTRUCTION_FUNCTION	(1 << 5)
#define		SC1602_INSTRUCTION_1LINE	(0 << 3)
#define		SC1602_INSTRUCTION_2LINE	(1 << 3)
#define		SC1602_INSTRUCTION_5x10DOTS	(1 << 2)
#define		SC1602_INSTRUCTION_5x7DOTS	(0 << 2)
#define SC1602_INSTRUCTION_FUNCTION_1LINE	\
	(SC1602_INSTRUCTION_FUNCTION | SC1602_INSTRUCTION_1LINE)
#define SC1602_INSTRUCTION_FUNCTION_2LINE	\
	(SC1602_INSTRUCTION_FUNCTION | SC1602_INSTRUCTION_2LINE)
#define		SC1602_INSTRUCTION_ADDRESS_SET	(1 << 7)
#define		SC1602_INSTRUCTION_LOCATE1	(0 << 6)
#define		SC1602_INSTRUCTION_LOCATE2	(1 << 6)
#define SC1602_INSTRUCTION_ADDRESS_LOCATE1	\
	(SC1602_INSTRUCTION_ADDRESS_SET | SC1602_INSTRUCTION_LOCATE1)
#define SC1602_INSTRUCTION_ADDRESS_LOCATE2	\
	(SC1602_INSTRUCTION_ADDRESS_SET | SC1602_INSTRUCTION_LOCATE2)

/* macros to get at IO space when running virtually */
#define IO_ADDRESS(x)	(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)
#define __io_address(a)     __io(IO_ADDRESS(a))

#define GPIOFSEL(x)  (0x00+(x)*4)
#define GPIOSET(x)   (0x1c+(x)*4)
#define GPIOCLR(x)   (0x28+(x)*4)

/* SC1602 device number define */
#define SC1602_DEVICE_COUNT	1
#define SC1602_MINOR		0
static int sc1602_major = 0;

/* GPIO data */
struct bcm2708_gpio {
	void __iomem *base;
};

/* SC1602 device data */
struct rasp_sc1602_device {
	struct device *class_dev;
	struct cdev cdev;
	struct class *class;
	struct bcm2708_gpio gpio;
	unsigned char entry_mode;
	unsigned char display;
	unsigned char line;
	unsigned char col;
	unsigned char line_mode;
	unsigned char dots;
	struct semaphore sem;
};

static struct rasp_sc1602_device *sc1602_device;

/* SC1602 driver private data */
struct rasp_sc1602_driver {
	struct timeval open_timeval;
};


/* GPIO function */

static void rasp_gpio_set(struct bcm2708_gpio *gpio, unsigned offset, int value)
{
	unsigned gpio_bank = offset / 32;
	unsigned gpio_field_offset = (offset - 32 * gpio_bank);

	if (offset >= RASP_GPIO_NUM_MAX)
		return;

	if (value)
		writel(1 << gpio_field_offset, 
		       gpio->base + GPIOSET(gpio_bank));
	else
		writel(1 << gpio_field_offset, 
		       gpio->base + GPIOCLR(gpio_bank));
}

static int rasp_gpio_set_function(struct bcm2708_gpio *gpio, unsigned offset,
				int function)
{
	unsigned gpiodir;
	unsigned gpio_bank = offset / 10;
	unsigned gpio_field_offset = (offset - 10 * gpio_bank) * 3;

	if (offset >= RASP_GPIO_NUM_MAX)
		return -EINVAL;

	gpiodir = readl(gpio->base + GPIOFSEL(gpio_bank));
	gpiodir &= ~(7 << gpio_field_offset);
	gpiodir |= function << gpio_field_offset;
	writel(gpiodir, gpio->base + GPIOFSEL(gpio_bank));

	gpiodir = readl(gpio->base + GPIOFSEL(gpio_bank));

	return 0;
}

static int rasp_gpio_dir_out(struct bcm2708_gpio *gpio, unsigned offset,
				int value)
{
	int ret;
	ret = rasp_gpio_set_function(gpio, offset, GPIO_FSEL_OUTPUT);
	if (ret >= 0)
		rasp_gpio_set(gpio, offset, value);
	return ret;
}


/* SC1602 function */

static void sc1602_set_databit(struct bcm2708_gpio *gpio, unsigned char d)
{
#define SC1602_DATABIT_MASK	((1 << sc1602_gpio_db4) | \
				 (1 << sc1602_gpio_db5) | \
				 (1 << sc1602_gpio_db6) | \
				 (1 << sc1602_gpio_db7))
	unsigned int data = (((d >> 0) & 0x01) << sc1602_gpio_db4) |
			    (((d >> 1) & 0x01) << sc1602_gpio_db5) |
			    (((d >> 2) & 0x01) << sc1602_gpio_db6) |
			    (((d >> 3) & 0x01) << sc1602_gpio_db7);

	unsigned gpio_bank = 0;	/* DATABIT gpio_bank is 0 only */

	writel((~data & SC1602_DATABIT_MASK), gpio->base + GPIOCLR(gpio_bank));
	writel((data & SC1602_DATABIT_MASK), gpio->base + GPIOSET(gpio_bank));

	rasp_gpio_set(gpio, sc1602_gpio_e, SC1602_DATA_ENABLE);
	udelay(20);	/* data setup >= 20us */
	rasp_gpio_set(gpio, sc1602_gpio_e, SC1602_DATA_DISABLE);
	udelay(20);	/* data hold and enable cycle*/
}

static void sc1602_output(struct bcm2708_gpio *gpio, unsigned char d, 
			unsigned char rs)
{
	rasp_gpio_set(gpio, sc1602_gpio_rs, rs);
	ndelay(40);	/* register setup >= 40ns */

	/* output up 4bit */
	sc1602_set_databit(gpio, ((d >> 4) & 0x0f));

	/* outout low 4bit */
	sc1602_set_databit(gpio, (d & 0x0f));
}

static void sc1602_locate(struct bcm2708_gpio *gpio, unsigned char line, 
			unsigned char col)
{
	unsigned char d;
	
	d = (line == 0) ? 
		SC1602_INSTRUCTION_ADDRESS_LOCATE1 :
		SC1602_INSTRUCTION_ADDRESS_LOCATE2;
	d |= col;
	sc1602_output(gpio, d, SC1602_REGISTER_SELECT_COMMAND);
}

static void sc1602_init(struct bcm2708_gpio *gpio)
{
	/* setup gpio direction */
	rasp_gpio_dir_out(gpio, sc1602_gpio_rs, 0);
	rasp_gpio_dir_out(gpio, sc1602_gpio_e, 0);
	rasp_gpio_dir_out(gpio, sc1602_gpio_db4, 0);
	rasp_gpio_dir_out(gpio, sc1602_gpio_db5, 0);
	rasp_gpio_dir_out(gpio, sc1602_gpio_db6, 0);
	rasp_gpio_dir_out(gpio, sc1602_gpio_db7, 0);

	mdelay(15);

	/* wakeup SC1602 */
	rasp_gpio_set(gpio, sc1602_gpio_rs,
		      SC1602_REGISTER_SELECT_COMMAND);
	ndelay(40);	/* register setup >= 40ns */
	sc1602_set_databit(gpio, 3);
	mdelay(5);

	rasp_gpio_set(gpio, sc1602_gpio_rs,
		      SC1602_REGISTER_SELECT_COMMAND);
	ndelay(40);	/* register setup >= 40ns */
	sc1602_set_databit(gpio, 3);
	mdelay(1);

	rasp_gpio_set(gpio, sc1602_gpio_rs,
		      SC1602_REGISTER_SELECT_COMMAND);
	ndelay(40);	/* register setup >= 40ns */
	sc1602_set_databit(gpio, 3);
	mdelay(1);

	/* set function set 4BIT-MODE */
	rasp_gpio_set(gpio, sc1602_gpio_rs,
		      SC1602_REGISTER_SELECT_COMMAND);
	ndelay(40);
	sc1602_set_databit(gpio, 2);
	mdelay(1);

	/* set function 2-LINE */
	sc1602_output(gpio, (SC1602_INSTRUCTION_FUNCTION_2LINE | 
			SC1602_INSTRUCTION_5x10DOTS), 
			SC1602_REGISTER_SELECT_COMMAND);
	mdelay(1);

	/* set function set DISPLAY-OFF */
	sc1602_output(gpio, SC1602_INSTRUCTION_DISPLAY_CONTROL_OFF, 
			SC1602_REGISTER_SELECT_COMMAND);
	mdelay(1);

	/* set function set DISPLAY-ON */
	sc1602_output(gpio, SC1602_INSTRUCTION_DISPLAY_CONTROL_ON, 
			SC1602_REGISTER_SELECT_COMMAND);
	mdelay(1);

	/* set function set iNCREMENT-MODE */
	sc1602_output(gpio, SC1602_INSTRUCTION_ENTRY_INCREMEMNT_MODE, 
			SC1602_REGISTER_SELECT_COMMAND);
	mdelay(1);

	/* set function CLEAR-DISPLAY */
	sc1602_output(gpio, SC1602_INSTRUCTION_CLEAR_DISPLAY, 
			SC1602_REGISTER_SELECT_COMMAND);
	mdelay(2);

	/* set function ADDRESS LOCATE */
	sc1602_locate(gpio, sc1602_device->line, sc1602_device->col);
}

static void sc1602_putchar(struct bcm2708_gpio *gpio, unsigned char d)
{
	sc1602_output(gpio, d, SC1602_REGISTER_SELECT_DATA);
}

static void sc1602_putstr(struct bcm2708_gpio *gpio, unsigned char *str, 
			int len)
{
	int i;
	for (i = 0; i < len; i++)
		sc1602_putchar(gpio, *(str++));
}


/* SC1602 driver function */

ssize_t rasp_sc1602_write(struct file *filp, const char __user *buf, 
			size_t count, loff_t *f_pos)
{
	int retval = 1;
	char *p;
	int len;
	unsigned char data[64];
	struct bcm2708_gpio *gpio = &sc1602_device->gpio;

	printk(KERN_DEBUG "%s:\n", __func__);

	if (down_interruptible(&sc1602_device->sem))
		return -ERESTARTSYS;

	len = (count > sizeof(data)) ? sizeof(data) : count;

	p = memchr(buf, '\n', count);
	len = p ? p - buf : count;

	if (len <= 0) {
		goto error;
	}

	if (copy_from_user(data, buf, len)) {
		retval = -EFAULT;
		goto error;
	}

	sc1602_putstr(gpio, data, len);

error:
	up(&sc1602_device->sem);

	return count;
}


static ssize_t sc1602_show(struct device *dev, struct device_attribute *attr, 
			char *buf)
{
	char *entry = (sc1602_device->entry_mode == 1) ?
		"increment" : "decrement";
	char *display = (sc1602_device->display == 1) ?
		"on" : "off";
	char *line =
	  (sc1602_device->line_mode == SC1602_INSTRUCTION_FUNCTION_1LINE) ?
		"1-line" : "2-line";

	printk(KERN_DEBUG "%s:\n", __func__);

	return sprintf(buf, "%s:entry_mode=%s, display=%s, line-mode=%s, line(locate)=%d, col(locate)=%d, \n", 
		attr->attr.name,
		entry,
		display,
		line,
		sc1602_device->line,
		sc1602_device->col);
}
static ssize_t sc1602_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	char *p;
	int len;

	printk(KERN_DEBUG "%s:", __func__);

	p = memchr(buf, '\n', count);
	len = p ? p - buf : count;

	if (len <= 0) {
		goto error;
	}

	if (0 == strncmp("clear-display", buf, len)) {
		sc1602_output(&sc1602_device->gpio, 
				SC1602_INSTRUCTION_CLEAR_DISPLAY, 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(2);
	} else if (0 == strncmp("cursor-at-home", buf, len)) {
		sc1602_output(&sc1602_device->gpio, 
				SC1602_INSTRUCTION_CURSOR_AT_HOME,
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} else if (0 == strncmp("increment-mode", buf, len)) {
		sc1602_device->entry_mode = 1;
		sc1602_output(&sc1602_device->gpio, 
				SC1602_INSTRUCTION_ENTRY_INCREMEMNT_MODE, 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} else if (0 == strncmp("decrement-mode", buf, len)) {
		sc1602_device->entry_mode = 0;
		sc1602_output(&sc1602_device->gpio, 
				SC1602_INSTRUCTION_ENTRY_DECREMENT_MODE, 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} else if (0 == strncmp("display-on", buf, len)) {
		sc1602_device->display = 1;
		sc1602_output(&sc1602_device->gpio, 
				SC1602_INSTRUCTION_DISPLAY_CONTROL_ON, 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} else if (0 == strncmp("display-off", buf, len)) {
		sc1602_device->display = 0;
		sc1602_output(&sc1602_device->gpio, 
				SC1602_INSTRUCTION_DISPLAY_CONTROL_OFF, 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	}
#ifdef NEVER
	else if (0 == strncmp("5x10dots", buf, len)) {
		sc1602_device->dots = SC1602_INSTRUCTION_5x10DOTS;
		sc1602_output(&sc1602_device->gpio, 
				(sc1602_device->line_mode | 
				 sc1602_device->dots), 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} else if (0 == strncmp("5x7dots", buf, len)) {
		sc1602_device->dots = SC1602_INSTRUCTION_5x7DOTS;
		sc1602_output(&sc1602_device->gpio, 
				(sc1602_device->line_mode | 
				 sc1602_device->dots), 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} 
#endif /* NEVER */
	else if (0 == strncmp("1-line", buf, len)) {
		sc1602_device->line_mode = SC1602_INSTRUCTION_FUNCTION_1LINE;
		sc1602_output(&sc1602_device->gpio, 
				(sc1602_device->line_mode | 
				 sc1602_device->dots), 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} else if (0 == strncmp("2-line", buf, len)) {
		sc1602_device->line_mode = SC1602_INSTRUCTION_FUNCTION_2LINE;
		sc1602_output(&sc1602_device->gpio, 
				(sc1602_device->line_mode | 
				 sc1602_device->dots), 
				SC1602_REGISTER_SELECT_COMMAND);
		mdelay(1);
	} else if (0 == strncmp("locate-1", buf, len)) {
		sc1602_device->line = 0;
		sc1602_locate(&sc1602_device->gpio, 
			       sc1602_device->line, sc1602_device->col);
	} else if (0 == strncmp("locate-2", buf, len)) {
		sc1602_device->line = 1;
		sc1602_locate(&sc1602_device->gpio, 
			       sc1602_device->line, sc1602_device->col);
	}

error:
	return count;
}

static struct device_attribute sc1602_class_attrs[] = {
	__ATTR(control, 0666, sc1602_show, sc1602_store),
	__ATTR_NULL,
};

int rasp_sc1602_open(struct inode *inode, struct file *filp)
{
	struct rasp_sc1602_driver *drv;

	printk(KERN_DEBUG "%s:\n", __func__);

	drv = kmalloc(sizeof(struct rasp_sc1602_driver), GFP_KERNEL);
	if (drv == NULL)
		return -ENOMEM;

	do_gettimeofday(&drv->open_timeval);
	filp->private_data = drv;

	return 0;
}

int rasp_sc1602_close(struct inode *inode, struct file *filp)
{
	struct rasp_sc1602_driver *drv = filp->private_data;
	struct timeval open_timeval, close_timeval;

	printk(KERN_DEBUG "%s:\n", __func__);

	open_timeval = drv->open_timeval;

	if (drv)
		kfree(drv);

	do_gettimeofday(&close_timeval);

	if (close_timeval.tv_usec < open_timeval.tv_usec)
		printk(KERN_DEBUG "open-close: %ld(s).%ld(us)\n", 
			close_timeval.tv_sec - open_timeval.tv_sec - 1, 
			1000000 + close_timeval.tv_usec - open_timeval.tv_usec);
	else
		printk(KERN_DEBUG "open-close: %ld(s).%ld(us)\n", 
			close_timeval.tv_sec - open_timeval.tv_sec, 
			close_timeval.tv_usec - open_timeval.tv_usec);

	return 0;
}

struct file_operations sc1602_fops = {
	.open = rasp_sc1602_open,
	.release = rasp_sc1602_close,
	.write = rasp_sc1602_write,
};

static int rasp_sc1602_init(void)
{
	dev_t dev = MKDEV(sc1602_major, SC1602_MINOR);
	int alloc_ret = 0;
	int major;
	int cdev_err = 0;

	sc1602_device = kmalloc(sizeof(struct rasp_sc1602_device), GFP_KERNEL);
	if (sc1602_device == NULL)
		return -ENOMEM;
	sc1602_device->gpio.base = __io_address(GPIO_BASE);
	sc1602_device->entry_mode = 1; /* increment-mode */
	sc1602_device->display = 1; /* display-on */
	sc1602_device->line = 0;
	sc1602_device->col = 0;
	sc1602_device->dots = SC1602_INSTRUCTION_5x10DOTS;
	sc1602_device->line_mode = SC1602_INSTRUCTION_FUNCTION_2LINE;

	alloc_ret = alloc_chrdev_region(&dev, 0, SC1602_DEVICE_COUNT, 
					"sc1602");
	if (alloc_ret)
		goto error;
	sc1602_major = major = MAJOR(dev);

	cdev_init(&sc1602_device->cdev, &sc1602_fops);
	sc1602_device->cdev.owner = THIS_MODULE;
	sc1602_device->cdev.ops = &sc1602_fops;
	cdev_err = cdev_add(&sc1602_device->cdev, 
			    MKDEV(sc1602_major, SC1602_MINOR), 1);
	if (cdev_err)
		goto error;

	/* register class */
	sc1602_device->class = class_create(THIS_MODULE, "sc1602");
	if (IS_ERR(sc1602_device->class)) {
		goto error;
	}
	sc1602_device->class->dev_attrs = sc1602_class_attrs;
	sc1602_device->class_dev = device_create(
		sc1602_device->class,
		NULL,
		MKDEV(sc1602_major, SC1602_MINOR),
		NULL,
		"sc1602%d",
		SC1602_MINOR
		);

	sema_init(&sc1602_device->sem, 1);

	sc1602_init(&sc1602_device->gpio);

	printk(KERN_DEBUG "sc1602 driver(major %d) installed\n", major);

	return 0;

error:
	if (cdev_err == 0)
		cdev_del(&sc1602_device->cdev);

	if (alloc_ret == 0)
		unregister_chrdev_region(dev, SC1602_DEVICE_COUNT);

	if (sc1602_device != NULL)
		kfree(sc1602_device);

	return -1;
}

static void rasp_sc1602_exit(void)
{
	dev_t dev = MKDEV(sc1602_major, 0);

	device_destroy(sc1602_device->class, 
		       MKDEV(sc1602_major, SC1602_MINOR));	
	class_destroy(sc1602_device->class);
	cdev_del(&sc1602_device->cdev);
	unregister_chrdev_region(dev, SC1602_DEVICE_COUNT);

	kfree(sc1602_device);

	printk(KERN_DEBUG "sc1602 driver removed\n");
}

module_init(rasp_sc1602_init);
module_exit(rasp_sc1602_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR( "Tadaki SAKAI" );
MODULE_DESCRIPTION("RaspberryPi SC1602 LCD driver");
MODULE_PARM_DESC(sc1602_gpio_rs, "RaspberryPi GPIO PIN number (connect SC1602 Regiser Select)");
MODULE_PARM_DESC(sc1602_gpio_e, "RaspberryPi GPIO PIN number (connect SC1602 Enable Signal)");
MODULE_PARM_DESC(sc1602_gpio_db4, "RaspberryPi GPIO PIN number (connect SC1602 DataBit4)");
MODULE_PARM_DESC(sc1602_gpio_db5, "RaspberryPi GPIO PIN number (connect SC1602 DataBit5)");
MODULE_PARM_DESC(sc1602_gpio_db6, "RaspberryPi GPIO PIN number (connect SC1602 DataBit6)");
MODULE_PARM_DESC(sc1602_gpio_db7, "RaspberryPi GPIO PIN number (connect SC1602 DataBit7)");
