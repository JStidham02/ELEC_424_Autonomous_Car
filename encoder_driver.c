#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>

/* Code written referencing course materials and https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c */

#define DEVICE_NAME "encoder_driver"
#define CLASS_NAME "elec424"

struct device *dev;
struct gpio_desc *button_desc;
int irq;

static volatile u64 last_time;
static volatile u64 curr_time;

static int major_number;
static struct class *encoder_driver_class = NULL;
static struct device *encoder_driver_device = NULL;
static int times_called = 0;
static char message[256] = {0};
static int last_diff = 0;
static short size_of_message;

static DEFINE_MUTEX(encoder_mutex);
static DEFINE_MUTEX(interrupt_mutex);

static int already_read;

/**
 * File operations declaration
 */
static int device_open(struct inode *, struct file *);
static ssize_t device_read(struct file *, char __user *, size_t, loff_t *);
static int device_release(struct inode *, struct file *);
/**
 * ISR Declaration
 */
static irq_handler_t encoder_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static struct file_operations fops = 
{
    .open = device_open,
    .read = device_read,
    .release = device_release,
};

static int __init hello_init(void){
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    encoder_driver_class = class_create(THIS_MODULE, CLASS_NAME);
    encoder_driver_device = device_create(encoder_driver_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    printk("init function has been called!\n");
    mutex_init(&encoder_mutex);
    return 0;
}

static void __exit hello_exit(void){
    device_destroy(encoder_driver_class, MKDEV(major_number,0));
    class_unregister(encoder_driver_class);
    class_destroy(encoder_driver_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    mutex_destroy(&encoder_mutex);
    printk("exit function has been called!\n");
}

static int device_open(struct inode *inodep, struct file *filep){
	if(!mutex_trylock(&encoder_mutex)){
		printk(KERN_ALERT "I'm being used!\n");
		return -EBUSY;
	}
    times_called++;
    printk("encoder character device opened!");
    return 0;
}

static ssize_t device_read(struct file *filep, char __user *buf, size_t length, loff_t *offset){
	long error_count;
	u64 call_time;
	u64 call_diff;
	if(already_read == 1)
	{
		return 0; //send EOF
	}
	already_read = 1;
	call_time = ktime_get_ns();
	call_diff = call_time - last_time;
	printk("User queried driver, call_diff = %lu, last_diff = %lu\n", (unsigned long) call_diff, (unsigned long) last_diff);
	while(mutex_is_locked(&interrupt_mutex));
	mutex_lock(&interrupt_mutex);
	if (call_diff > 3*last_diff)
	{
		printk("Updating message because of call_diff\n!");
		last_diff = call_diff;
		sprintf(message, "%lu\n", (unsigned long) call_diff);
		size_of_message = strlen(message);
	}
	error_count = copy_to_user(buf, message, size_of_message);
	mutex_unlock(&interrupt_mutex);
	printk("Sent %d characters to user!\n", size_of_message);
	printk("User should have received message: %s\n", message);
	return size_of_message;
}

static int device_release(struct inode *inodep, struct file *filep)
{
	mutex_unlock(&encoder_mutex);
	already_read = 0;
	printk("Device released!\n");
	return 0;
}

/**
 * Probe function
 */
static int encoder_probe(struct platform_device *pdev)
{
	//declare variables
	int ret;
	mutex_init(&interrupt_mutex);
	printk("Starting probe function!\n");
	hello_init();
	//get device
	dev = &(pdev->dev);
	//verify device
	if(dev == NULL)
	{
		printk("Failed to get dev\n");
		return -1;
	}
	// get descriptors for pins
	button_desc = devm_gpiod_get(dev, "encoder", GPIOD_IN);
	if(button_desc == NULL)
	{
		printk("Failed to get encoder gpio descriptor\n");
		return -1;
	}
	//set debounce
	gpiod_set_debounce(button_desc, 1200000);
	// get irq number
	irq = gpiod_to_irq(button_desc);
	if(irq < 0)
	{	
		printk("Failed to get IRQ number\n");
		return -1;
	}
	//assign irq
	ret = request_irq((unsigned int) irq, (irq_handler_t) encoder_irq_handler, IRQF_TRIGGER_RISING, "anything", NULL);
	if(ret < 0)
	{
		printk("Failed to install irq\n");
		return -1;
	}
	last_time = ktime_get_ns();
	//should read 0 ns
	sprintf(message, "%lu\n", (unsigned long) last_time);
	size_of_message = strlen(message);
	printk("Initial message reads %s!\n", message);
	last_diff = last_time;
	already_read = 0;
	//print installation message
	printk("Driver Installed!\n");
	return 0;
}

// remove function
static int encoder_remove(struct platform_device *pdev)
{
	free_irq(irq, NULL);
	printk("Removed driver!\n");	
	hello_exit();
	return 0;
}

// set compatibale to get leds from device tree
static struct of_device_id matchy_match[] = {
    {.compatible = "encoder-driver"},
    {/* leave alone - keep this here (end node) */},
};

// platform driver object
static struct platform_driver adam_driver = {
	.probe	 = encoder_probe,
	.remove	 = encoder_remove,
	.driver	 = {
	       .name  = "The Rock: this name doesn't even matter",
	       .owner = THIS_MODULE,
	       .of_match_table = matchy_match,
	},
};

module_platform_driver(adam_driver);

// define interrupt handler
static irq_handler_t encoder_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
	//print message
	u64 diff;
	printk("encoder_irq: Encoder interrupt triggered!\n");
	curr_time = ktime_get_ns();
	diff = curr_time - last_time;
	if (diff > 1000000)
	{
		//only count when greater than 1 ms
		//set last to curr
		while(mutex_is_locked(&interrupt_mutex));
		mutex_lock(&interrupt_mutex);
		last_time = curr_time;
		sprintf(message, "%lu\n", (unsigned long) diff);
		mutex_unlock(&interrupt_mutex);
		size_of_message = strlen(message);
		printk("Detected an encode press, time difference was %lu nanoseconds", (unsigned long) diff);
		last_diff = diff;
	}
	return (irq_handler_t) IRQ_HANDLED;
}

MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("Josh Stidham");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");
MODULE_VERSION("0.000001");
