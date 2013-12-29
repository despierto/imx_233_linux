/*
 * Driver for i.mx233 internal die temperature sensor
 *
 * 2010 Alexander Kudjashev
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/hwmon.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <asm/processor.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <mach/ddi_bc.h>
#include <mach/lradc.h>
#include <mach/regs-power.h>
#include <mach/regs-lradc.h>


#define KELVIN_TO_CELSIUS_CONST (273*1000)
#define GAIN_CORRECTION 1012

struct imx233ct {
    struct device *hwmon_dev;
    struct mutex lock;
};

/*
 * Use the the lradc1 channel
 * get the die temperature from on-chip sensor.
 */
int MeasureInternalDieTemperature(void)
{
    uint32_t  ch8Value, ch9Value;

    /* power up internal tep sensor block */
    __raw_writel(BM_LRADC_CTRL2_TEMPSENSE_PWD,
            REGS_LRADC_BASE + HW_LRADC_CTRL2_CLR);

    /* mux to the lradc 8th temp channel */
    __raw_writel(BF(0xF, LRADC_CTRL4_LRADC1SELECT),
            REGS_LRADC_BASE + HW_LRADC_CTRL4_CLR);
    __raw_writel(BF(8, LRADC_CTRL4_LRADC1SELECT),
            REGS_LRADC_BASE + HW_LRADC_CTRL4_SET);

    /* Clear the interrupt flag */
    __raw_writel(BM_LRADC_CTRL1_LRADC1_IRQ,
            REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);
    __raw_writel(BF(1 << LRADC_CH1, LRADC_CTRL0_SCHEDULE),
            REGS_LRADC_BASE + HW_LRADC_CTRL0_SET);

    /* Wait for conversion complete*/
    while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1)
            & BM_LRADC_CTRL1_LRADC1_IRQ))
                cpu_relax();

    /* Clear the interrupt flag again */
    __raw_writel(BM_LRADC_CTRL1_LRADC1_IRQ,
            REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);

    /* read temperature value and clr lradc */
    ch8Value = __raw_readl(REGS_LRADC_BASE + HW_LRADC_CHn(LRADC_CH1))
                & BM_LRADC_CHn_VALUE;

    __raw_writel(BM_LRADC_CHn_VALUE,
            REGS_LRADC_BASE + HW_LRADC_CHn_CLR(LRADC_CH1));

    /* mux to the lradc 9th temp channel */
    __raw_writel(BF(0xF, LRADC_CTRL4_LRADC1SELECT),
            REGS_LRADC_BASE + HW_LRADC_CTRL4_CLR);
    __raw_writel(BF(9, LRADC_CTRL4_LRADC1SELECT),
            REGS_LRADC_BASE + HW_LRADC_CTRL4_SET);

    /* Clear the interrupt flag */
    __raw_writel(BM_LRADC_CTRL1_LRADC1_IRQ,
            REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);
    __raw_writel(BF(1 << LRADC_CH1, LRADC_CTRL0_SCHEDULE),
            REGS_LRADC_BASE + HW_LRADC_CTRL0_SET);
    /* Wait for conversion complete */
    while (!(__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1)
            & BM_LRADC_CTRL1_LRADC1_IRQ))
                cpu_relax();

    /* Clear the interrupt flag */
    __raw_writel(BM_LRADC_CTRL1_LRADC1_IRQ,
            REGS_LRADC_BASE + HW_LRADC_CTRL1_CLR);
    /* read temperature value */
    ch9Value = __raw_readl(REGS_LRADC_BASE + HW_LRADC_CHn(LRADC_CH1))
                & BM_LRADC_CHn_VALUE;

    __raw_writel(BM_LRADC_CHn_VALUE,
            REGS_LRADC_BASE + HW_LRADC_CHn_CLR(LRADC_CH1));

    /* power down temp sensor block */
    __raw_writel(BM_LRADC_CTRL2_TEMPSENSE_PWD,
            REGS_LRADC_BASE + HW_LRADC_CTRL2_SET);

    return ((ch9Value-ch8Value)*GAIN_CORRECTION/4 - KELVIN_TO_CELSIUS_CONST);
}

static ssize_t imx233ct_sense_temp(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct imx233ct *imx233ct_data = dev_get_drvdata(dev);
    int status;
    int val;

    if (mutex_lock_interruptible(&imx233ct_data->lock))
        return -ERESTARTSYS;

    val = MeasureInternalDieTemperature();
    status = sprintf(buf, "%d\n", val);

    mutex_unlock(&imx233ct_data->lock);
    return status;
}

static DEVICE_ATTR(temp1_input, S_IRUGO, imx233ct_sense_temp, NULL);

static ssize_t imx233ct_show_name(struct device *dev, struct device_attribute
                  *devattr, char *buf)
{

    return sprintf(buf, "%s\n", "imx233ct");
}

static DEVICE_ATTR(name, S_IRUGO, imx233ct_show_name, NULL);

static int __init imx233ct_probe(struct platform_device *pdev)
{
    struct imx233ct *imx233ct_data;
    int status;

    imx233ct_data = kzalloc(sizeof *imx233ct_data, GFP_KERNEL);
    if (!imx233ct_data)
        return -ENOMEM;

    mutex_init(&imx233ct_data->lock);

    /* sysfs hook */
    imx233ct_data->hwmon_dev = hwmon_device_register(&pdev->dev);
    if (IS_ERR(imx233ct_data->hwmon_dev)) {
        dev_dbg(&pdev->dev, "hwmon_device_register failed.\n");
        status = PTR_ERR(imx233ct_data->hwmon_dev);
        goto out_dev_reg_failed;
    }
    platform_set_drvdata(pdev, imx233ct_data);

    if ((status = device_create_file(&pdev->dev, &dev_attr_temp1_input))
     || (status = device_create_file(&pdev->dev, &dev_attr_name))) {
        dev_dbg(&pdev->dev, "device_create_file failure.\n");
        goto out_dev_create_file_failed;
    }

    return 0;

out_dev_create_file_failed:
    device_remove_file(&pdev->dev, &dev_attr_temp1_input);
    hwmon_device_unregister(imx233ct_data->hwmon_dev);
out_dev_reg_failed:
    platform_set_drvdata(pdev, NULL);
    kfree(imx233ct_data);

    return status;
}

static int imx233ct_remove(struct platform_device *pdev)
{
    struct imx233ct *imx233ct_data = platform_get_drvdata(pdev);

    hwmon_device_unregister(imx233ct_data->hwmon_dev);
    device_remove_file(&pdev->dev, &dev_attr_temp1_input);
    device_remove_file(&pdev->dev, &dev_attr_name);
    platform_set_drvdata(pdev, NULL);
    kfree(imx233ct_data);

    return 0;
}

static struct platform_driver imx233ct_driver = {
    .driver     = {
        .name   = "imx233ct",
    },
    .probe      = imx233ct_probe,
    .remove     = imx233ct_remove,
};

static struct platform_device *imx233ct_device;

static int __init imx233ct_init(void)
{
    int ret = 0;

    ret = platform_driver_register(&imx233ct_driver);

    if (!ret) {
        imx233ct_device = platform_device_alloc("imx233ct", 0);

        if (imx233ct_device)
            ret = platform_device_add(imx233ct_device);
        else
            ret = -ENOMEM;

        if (ret) {
            platform_device_put(imx233ct_device);
            platform_driver_unregister(&imx233ct_driver);
        }
    }

    return ret;
}

static void __exit imx233ct_exit(void)
{
    platform_device_unregister(imx233ct_device);
    platform_driver_unregister(&imx233ct_driver);
}

module_init(imx233ct_init);
module_exit(imx233ct_exit);


MODULE_AUTHOR("Alexander Kudjashev");
MODULE_DESCRIPTION("Driver for i.mx233 internal die temperature sensor");
MODULE_LICENSE("GPL");
