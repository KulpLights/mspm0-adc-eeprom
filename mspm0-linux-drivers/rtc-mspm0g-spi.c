/*
rtc-mspm0g-spi.c - RTC driver for TI MSPM0G microcontroller devices
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/slab.h>


/*RTC registers are abritarily defined and can be changed: using sub-addresses in rtc.c*/
#define MSPM0G_RTC_ID       0x00
#define MSPM0G_RTC_WKINT    0x01
#define MSPM0G_RTC_INTCFG   0x02, 0x03
#define MSPM0G_RTC_INTFLG   0x04, 0x05
#define MSPM0G_RTC_OUTCFG   0x06

#define MSPM0G_REG_SECS     0x07        /*00 - 59*/
#define MSPM0G_REG_MIN      0x08        /*00 - 59*/
#define MSPM0G_REG_HOUR     0x09        /*00 - 24*/
#define MSPM0G_REG_WDAY     0x0A        /*00 - 6*/
#define MSPM0G_REG_MDAY     0x0B        /*00 - 32*/
#define MSPM0G_REG_MONTH    0x0C        /*00 - 11*/
#define MSPM0G_REG_YEAR     0x0D, 0x0E

#define MSPM0G_RTC_ALRM_INT 0x0F
#define MSPM0G_RTC_ALRM_ST  0x10

#define MSPM0G_ALRM_SEC     0x13
#define MSPM0G_ALRM_MIN     0x14
#define MSPM0G_ALRM_HOUR    0x15
#define MSPM0G_ALRM_WDAY    0x16
#define MSPM0G_ALRM_MDAY    0x17
#define MSPM0G_ALRM_MONTH   0x18
#define MSPM0G_ALRM_YEAR    0x19,0x1A


struct mspm0g {
    struct rtc_device *rtc;
    struct regmap *map;
    int irq;
};

static int mspm0g_get_time(struct device *dev, struct rtc_time *dt){
    struct mspm0g   *mspm0g = dev_get_drvdata(dev);
    int century, err;
    uint8_t regs[8];

    //printk("reading Register: 0x%x\n", MSPM0G_REG_SECS);
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

// NOT IMPLEMENTED
static int mspm0g_read_alarm(struct device *dev, struct rtc_wkalrm *alarm){
    struct mspm0g *mspm0g = dev_get_drvdata(dev);
    uint8_t regs[4];
    int ctl, err;
    if (mspm0g->irq <=0)
        return -EINVAL;

    // Read Control registers
    err = regmap_read(mspm0g->map, MSPM0G_RTC_ALRM_INT, &ctl);
    if(err)
        return err;
    
    alarm->enabled = clt [0]

    //Read alarm registers
    err = regmap_read(mspm0g->map, MSPM0G_ALRM_MIN, regs);
    if(err)
        return err;
    
    alarm->time.tm_sec = -1;
    alarm->time.tm_min = bcd2bin(regs[0] & 0x7f);
    alarm->time.tm_hour = bcd2bin(regs[1] & 0x7f);
    alarm->time.tm_wday = bcd2bin(regs[2] & 0x07);
    alarm->time.tm_mday = bcd2bin(regs[3] & 0x7f);
    alarm->time.tm_mon = -1;
    alarm->time.tm_yday = -1;
    alarm->time.is_dst = -1;

    dev_dbg(dev, "%s, sec=%d min=%d hour=%d wday=%d mday=%d mon=%d enabled=%d\n",
		__func__, t->time.tm_sec, t->time.tm_min, t->time.tm_hour,
		t->time.tm_wday, t->time.tm_mday, t->time.tm_mon, t->enabled);


    return 0;
}

// NOT IMPLEMENTED
static int mspm0g_set_alarm(struct device *dev, struct rtc_wkalrm *alarm){
    struct mspm0g *mspm0g = dev_get_drvdata(dev);

}

// NOT IMPLEMENTED
static irqreturn_t mspm0g_irq(int irq, void *dev_id){
    struct mspm0g   *mspm0g = dev_id;
    struct mutex    *lock = &mspm0g->rtc->op_lock;
    int reg, err;

    mutex_lock(lock);

    /*Read Alarm Status Registers*/
    err = regmap_read(mspm0g->map, MSPM0G_RTC_ALRM_ST, &reg);
    if(err)
        goto out;

    if(!)

    //turn off alarm 
    err = regmap_update_bits(mspm0g->)


out:
    mutex_unlock(lock);
    return IRQ_HANDLED;

}

static const struct rtc_class_ops mspm0g_rtc_ops = {
    .read_time          = mspm0g_get_time,
    .set_time           = mspm0g_set_time,
    //.read_alarm         = mspm0g_read_alarm,
    //.set_alarm          = mspm0g_set_alarm,
    //.alarm_irq_enable   = mspm0g_alarm_irq_enable,
};

static const struct of_device_id mspm0g_of_match[] = {
    {.compatible = "ti,mspm0g_spi",},
};

static int mspm0g_probe(struct spi_device *spi){
   
    struct mspm0g           *mspm0g;
    struct regmap_config    config = {.reg_bits = 8, .val_bits = 8, .write_flag_mask = 0x80, };
    int                     err = -ENODEV;
    unsigned int            regs[8];
    
    printk("Begin Probe\n");
    
    //Allocate memory for mspm0g virtual device struct
    mspm0g = devm_kzalloc(&spi->dev, sizeof(mspm0g), GFP_KERNEL);
    if(!mspm0g)
        return -ENOMEM;
    
/*----------------------- SPI initialization -----------------------*/
    //set up spi mode 0
    spi->mode |= SPI_MODE_0;
    spi->max_speed_hz = 100000;
    spi->bits_per_word = 8;
    err = spi_setup(spi);
    if(err){
        dev_err(&spi->dev, "spi setup failed for rtc mspm0g\n");
        return err;
    }
    spi_set_drvdata(spi, mspm0g);

    mspm0g->map = devm_regmap_init_spi(spi, &config);
    if(IS_ERR(mspm0g->map)){
        dev_err(&spi->dev, "spi regmap allocation failed for rtc mspm0g\n");
        return PTR_ERR(mspm0g->map);
    }

    //perform a test spi read
    err = regmap_read(mspm0g->map, MSPM0G_RTC_ID, regs);
    if(err)
        return err;
    

    //DO RTC CONTROL SETUP HERE
    //Check output mode of hours (12 vs 24)
    //set time output mode to BCD

/*----------------------- RTC Initialization ---------------------*/
    mspm0g->rtc = devm_rtc_allocate_device(&spi->dev);
    if(IS_ERR(mspm0g->rtc))
        return PTR_ERR(mspm0g->rtc);
    
    mspm0g->rtc->ops = &mspm0g_rtc_ops;
    mspm0g->rtc->range_min = RTC_TIMESTAMP_BEGIN_0000;
    mspm0g->rtc->range_max = RTC_TIMESTAMP_END_9999;

    err = rtc_register_device(mspm0g->rtc);
    if (err)
        return err;

/*----------------------- IRQ Initialization ---------------------*/


/*----------------------- NVRAM Initialization ---------------------*/

    return 0;
}

static struct spi_driver mspm0g_driver = {
    .driver = {
        .name = "rtc-mspm0g-spi",
        .of_match_table = mspm0g_of_match,
    },
    .probe = mspm0g_probe,
};
module_spi_driver(mspm0g_driver);


MODULE_DESCRIPTION("I2C driver for interfacing MSPM0G devices");
MODULE_LICENSE("GPL");