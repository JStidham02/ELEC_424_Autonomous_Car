#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
/* Code written referencing course materials and https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c */

/* YOU WILL HAVE TO DECLARE SOME VARIABLES HERE */
struct device *dev;
struct gpio_desc *button_desc;
int irq;

/**
 * ISR Declaration
 */
static irq_handler_t encoder_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);


/**
 * Probe function
 */
static int encoder_probe(struct platform_device *pdev)
{
	//declare variables
	int ret;
	//get device
	dev = &(pdev->dev);
	//verify device
	if(dev == NULL)
	{
		printk("Failed to get dev\n");
		return -1;
	}
	// get descriptors for pins
	//TODO change this to proper pin for encoder
	button_desc = devm_gpiod_get(dev, "encoder", GPIOD_IN);
	if(button_desc == NULL)
	{
		printk("Failed to get encoder gpio descriptor\n");
		return -1;
	}
	//set debounce
	gpiod_set_debounce(button_desc, 1000000);
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
	//print installation message
	printk("Driver Installed!\n");
	return 0;
}

// remove function
static int encoder_remove(struct platform_device *pdev)
{
	free_irq(irq, NULL);
	printk("Removed driver!\n");
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
	printk("encoder_irq: Encoder interrupt triggered!\n");
	//TODO do timing here
	return (irq_handler_t) IRQ_HANDLED;
}

MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("Josh Stidham");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");
