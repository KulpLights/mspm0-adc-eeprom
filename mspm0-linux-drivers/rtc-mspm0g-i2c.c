/*
rtc-mspm0g-i2c.c - RTC driver for TI MSPM0G microcontroller devices
*/

#include <linux/acpi.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

/*RTC registers are abritarily defined and can be changed: using sub-addresses in rtc.c*/
#define MSPM0G_RTC_ID       0x00
#define MSPM0G_RTC_WKINT    0x01
#define MSPM0G_RTC_INTCFG   0x02

#define MSPM0G_REG_SECS     0x07        /*00 - 59*/
#define MSPM0G_REG_MIN      0x08        /*00 - 59*/
#define MSPM0G_REG_HOUR     0x09        /*00 - 24*/
#define MSPM0G_REG_WDAY     0x0A        /*00 - 6*/
#define MSPM0G_REG_MDAY     0x0B        /*00 - 32*/
#define MSPM0G_REG_MONTH    0x0C        /*00 - 11*/
#define MSPM0G_REG_YEAR     0x0D

struct mspm0g {
    struct rtc_device *rtc;
    struct regmap *map;
    int irq;
};

static int mspm0g_get_time(struct device *dev, struct rtc_time *dt){
    struct mspm0g   *mspm0g = dev_get_drvdata(dev);
    int hbyear, err;
    uint8_t regs[8];

    err = regmap_bulk_read(mspm0g->map, MSPM0G_REG_SECS, regs, 8);
    if(err) {
        dev_err(dev, "%s error %d\n", "read", err);
        return err;
    }

    dt->tm_sec      = regs[0] & 0x3F;
    dt->tm_min      = regs[1] & 0x3F;
    dt->tm_hour     = regs[2] & 0x1F;
    dt->tm_wday     = regs[3] & 0x07;
    dt->tm_mday     = regs[4] & 0x1F;
    dt->tm_mon      = regs[5] & 0x0F;
    
    dt->tm_year     = ((regs[6] & 0x0F) << 8) + (regs[7] & 0xFF);
    return 0;
}

static int mspm0g_set_time(struct device *dev, struct rtc_time *dt){
    struct mspm0g *mspm0g = dev_get_drvdata(dev);
    int err;
    uint8_t regs[8];

    regs[0] = dt->tm_sec;
    regs[1] = dt->tm_min;
    regs[2] = dt->tm_hour;
    regs[3] = dt->tm_wday;
    regs[4] = dt->tm_mday;
    regs[5] = dt->tm_mon;

    regs[6] = (uint8_t)((dt->tm_year >> 8) & 0x000F);
    regs[7] = (uint8_t)(dt->tm_year & 0xFF);

    err = regmap_bulk_write(mspm0g->map, MSPM0G_REG_SECS, regs, 8);
    if(err)
        return err;

    return 0;
}

static const struct rtc_class_ops mspm0g_rtc_ops = {
    .read_time          = mspm0g_get_time,
    .set_time           = mspm0g_set_time,
};

static const struct of_device_id mspm0g_of_match[] = {
    {.compatible = "ti,mspm0g_i2c",},
};

static int mspm0g_probe(struct i2c_client *client, const struct i2c_device_id *id){
   
    struct mspm0g          *mspm0g;
    struct regmap_config    config = {.reg_bits = 8, .val_bits = 8,};
    int                     err = -ENODEV;
    unsigned int           regs[8];

    //Allocate memory for mspm0g virtual device struct
    mspm0g = devm_kzalloc(&client->dev, sizeof(mspm0g), GFP_KERNEL);
    if(!mspm0g)
        return -ENOMEM;

    dev_set_drvdata(&client->dev, mspm0g);
    
/*----------------------- I2C initialization -----------------------*/
    //initialize the size of each register being accessed in I2C calls
    mspm0g->map = devm_regmap_init_i2c(client, &config);
    if(IS_ERR(mspm0g->map)){
        dev_err(&client->dev, "regmap allocation failed for rtc mspm0g\n");
        return PTR_ERR(mspm0g->map);
    }
    i2c_set_clientdata(client, mspm0g);

    //perform a test i2c read
    err = regmap_read(mspm0g->map, MSPM0G_RTC_ID, regs);
    if(err)
        return err;
    

    //DO RTC CONTROL SETUP HERE
    //Check which mode the hours are in

/*----------------------- RTC Initialization ---------------------*/
    mspm0g->rtc = devm_rtc_allocate_device(&client->dev);
    if(IS_ERR(mspm0g->rtc))
        return PTR_ERR(mspm0g->rtc);
    
    mspm0g->rtc->ops = &mspm0g_rtc_ops;
    mspm0g->rtc->range_min = RTC_TIMESTAMP_BEGIN_0000;
    mspm0g->rtc->range_max = RTC_TIMESTAMP_END_9999;

    err = rtc_register_device(mspm0g->rtc);
    if (err)
        return err;

/*----------------------- IRQ Initialization ---------------------*/


    return 0;
}

static struct i2c_driver mspm0g_driver = {
    .driver = {
        .name = "rtc-mspm0g-i2c",
        .of_match_table = mspm0g_of_match,
    },
    .probe = mspm0g_probe,
};
module_i2c_driver(mspm0g_driver);


MODULE_DESCRIPTION("I2C driver for interfacing MSPM0G devices");
MODULE_LICENSE("GPL");