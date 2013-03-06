/*
 * BQ275x0 battery driver
 *
 * Copyright (C) 2010 Audi PC Huang <AudiPCHuang@fihtdc.com>
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/mfd/pmic8058.h> /* for controlling pmic8058 GPIO */ 
#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <mach/bq275x0_battery.h>
#include <mach/ds2784.h>
#include <mach/irqs.h>

#define BATTERY_POLLING_TIMER   30000
#define BATTERY_POLLING_TIMER2  90000

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq275x0_device_info {
    struct device       *dev;
    int         id;
    int         voltage_uV;
    int         current_uA;
    int         temp_C;
    int         charge_rsoc;
    int         health;
    int         present;
    struct power_supply bat;
    
    int     polling_interval;

    unsigned    pmic_BATLOW;
    unsigned    pmic_BATGD;
    
    bool        dev_ready;
    bool        chg_inh;

    struct timer_list polling_timer;
    struct work_struct bq275x0_BATGD;
    struct work_struct bq275x0_BATLOW;
    struct work_struct bq275x0_update;

    struct i2c_client   *client;
    
    struct wake_lock bq275x0_wakelock;
    
    short chg_inh_temp_h;
    short chg_inh_temp_l;
};

struct manufacturer_info {
    u8 dfi_ver;
    char prj_name[6];
    u8 date[4];
    u8 other[21];
};

static enum power_supply_property bq275x0_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_TECHNOLOGY,
};

static struct bq275x0_device_info *di;
extern u32 msm_batt_get_batt_status(void);
extern int msm_batt_OT_protection(void);
extern void msm_batt_notify_battery_full(void);

static void bq275x0_endian_adjust(u8* buf, int len)
{
    int i;
    u8 temp[4];
    
    for (i = 0; i < len; i++)
        temp[(len - 1) - i] = buf[i];
    for (i = 0; i < len; i++)
        buf[i] = temp[i];
}

/*
 * bq275x0 specific code
 */
static int bq275x0_read(struct bq275x0_device_info *di, u8 cmd, u8 *data, int length)
{
    int ret;
    
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = di->client->addr,
            .flags  = 0,
            .buf    = (void *)&cmd,
            .len    = 1
        },
        [1] = {
            .addr   = di->client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)data,
            .len    = length
        }
    };

    ret = i2c_transfer(di->client->adapter, msgs, 2);
    mdelay(1);
    return (ret < 0) ? -1 : 0;    
}

static int bq275x0_write(struct bq275x0_device_info *di, u8 *cmd, int length)
{
    int ret;
    
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = di->client->addr,
            .flags  = 0,
            .buf    = (void *)cmd,
            .len    = length
        },
    };

    ret = i2c_transfer(di->client->adapter, msgs, 1);
    mdelay(16);
    return (ret < 0) ? -1 : 0;    
}

static int bq275x0_battery_control_command(u16 subcommand, u8* data, int len)
{
    u8 cmd[3];
    
    cmd[0] = BQ275X0_CMD_CNTL;
    memcpy(&cmd[1], &subcommand, sizeof(u16));
    
    if (bq275x0_write(di, cmd, sizeof(cmd)))
        return -1;
    if (len != 0)
        if (bq275x0_read(di, BQ275X0_CMD_CNTL, data, len))
            return -1;
    
    return 0;
}

int bq275x0_battery_access_data_flash(u8 subclass, u8 offset, u8* buf, int len, bool write)
{
    int i       = 0;
    unsigned sum= 0;
    u8 cmd[33];
    u8 block_num= offset / 32;
    u8 cmd_code = BQ275X0_EXT_CMD_DFD + offset % 32;
    u8 checksum = 0;
    
    /* Set SEALED/UNSEALED mode. */
    cmd[0] = BQ275X0_EXT_CMD_DFDCNTL;
    cmd[1] = 0x00;
    if (bq275x0_write(di, cmd, 2))
        return -1;
    
    /* Set subclass */
    cmd[0] = BQ275X0_EXT_CMD_DFCLS;
    cmd[1] = subclass;
    if (bq275x0_write(di, cmd, 2))
        return -1;
    
    /* Set block to R/W */
    cmd[0] = BQ275X0_EXT_CMD_DFBLK;
    cmd[1] = block_num;
    if (bq275x0_write(di, cmd, 2))
        return -1;

    if (write) {
        /* Calculate checksum */
        for (i = 0; i < len; i++)
            sum += buf[i];
        checksum = 0xFF - (sum & 0x00FF);
        
        /* Set checksum */
        cmd[0] = BQ275X0_EXT_CMD_DFDCKS;
        cmd[1] = checksum;
        if (bq275x0_write(di, cmd, 2))
            return -1;

        /* Write data */
        cmd[0] = cmd_code;
        memcpy((cmd + 1), buf, len);
        if (bq275x0_write(di, cmd, (len + 1)))
            return -1;
    } else {
        /* Read data */
        if (bq275x0_read(di, cmd_code, buf, len))
            return -1;
    }
    
    return 0;
}

static u16 bq275x0_battery_flags(void)
{
    int ret;
    u16 flags = 0;

    ret = bq275x0_read(di, BQ275X0_CMD_FLAGS, (u8*)&flags, sizeof(u16));
    if (ret < 0) {
        dev_err(di->dev, "error reading flags\n");
        return 0;
    }

    return flags;
}

/*
 * Return the battery temperature in Celsius degrees
 * Or < 0 if something fails.
 */
int bq275x0_battery_temperature(void)
{
    int ret;
    s16 temp = 0;

    ret = bq275x0_read(di, BQ275X0_CMD_TEMP, (u8*)&temp, sizeof(s16));
    if (ret) {
        dev_err(di->dev, "error reading temperature\n");
        return ret;
    }

    return temp - (273 * 10);
}
EXPORT_SYMBOL(bq275x0_battery_temperature);

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
int bq275x0_battery_voltage(void)
{
    int ret;
    s16 volt = 0;

    ret = bq275x0_read(di, BQ275X0_CMD_VOLT, (u8*)&volt, sizeof(s16));
    if (ret) {
        dev_err(di->dev, "error reading voltage\n");
        return ret;
    }

    return volt;
}
EXPORT_SYMBOL(bq275x0_battery_voltage);

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
int bq275x0_battery_current(void)
{
    int ret;
    s16 curr = 0;
    //u16 flags = 0;

    ret = bq275x0_read(di, BQ275X0_CMD_AI, (u8*)&curr, sizeof(s16));
    if (ret) {
        dev_err(di->dev, "error reading current\n");
        return 0;
    }
    /*ret = bq275x0_read(di, BQ275X0_CMD_FLAGS, (u8*)&flags, sizeof(s16));
    if (ret < 0) {
        dev_err(di->dev, "error reading flags\n");
        return 0;
    }
    if ((flags & (1 << BQ275X0_FLAGS_CHG)) != 0) {
        dev_dbg(di->dev, "negative current!\n");
        return -curr;
    }*/
    return curr;
}
EXPORT_SYMBOL(bq275x0_battery_current);

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
int bq275x0_battery_soc(void)
{
    int ret;
    u16 soc = 0;

    ret = bq275x0_read(di, BQ275X0_CMD_SOC, (u8*)&soc, sizeof(s16));
    if (ret) {
        dev_err(di->dev, "error reading relative State-of-Charge\n");
        return ret;
    }

    return soc;
}
EXPORT_SYMBOL(bq275x0_battery_soc);

/*
 * Return the battery health
 */
int bq275x0_battery_health(void)
{
    if (!di->present)
        return POWER_SUPPLY_HEALTH_UNKNOWN;
    if (!(di->chg_inh_temp_h > di->temp_C))
        return POWER_SUPPLY_HEALTH_OVERHEAT;
    if (!(di->chg_inh_temp_l < di->temp_C))
        return POWER_SUPPLY_HEALTH_COLD;
        
    return POWER_SUPPLY_HEALTH_GOOD;
}
EXPORT_SYMBOL(bq275x0_battery_health);

int bq275x0_battery_fw_version(void)
{
    int ret;
    u16 fw_version = 0;

    ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_FW_VERSION,
                                            (u8*)&fw_version,
                                            sizeof(u16)
                                            );
    if (ret) {
        dev_err(di->dev, "error reading firmware version\n");
        return ret;
    }

    return fw_version;
}
EXPORT_SYMBOL(bq275x0_battery_fw_version);

int bq275x0_battery_device_type(void)
{
    int ret;
    u16 device_type;

    ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_DEVICE_TYPE,
                                            (u8*)&device_type,
                                            sizeof(u16)
                                            );
    if (ret) {
        dev_err(di->dev, "error reading firmware version\n");
        return ret;
    }

    return device_type;
}
EXPORT_SYMBOL(bq275x0_battery_device_type);

int bq275x0_battery_IT_enable(void)
{
    int ret;

    ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_IT_ENABLE,
                                            NULL,
                                            0
                                            );
    if (ret) {
        dev_err(di->dev, "error IT Enable\n");
        return ret;
    }

    dev_info(di->dev, "End Gauge IT Enable\n");
    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_IT_enable);

int bq275x0_battery_reset(void)
{
    int ret;

    ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_RESET,
                                            NULL,
                                            0
                                            );
    if (ret) {
        dev_err(di->dev, "error reset\n");
        return ret;
    }
    
    dev_info(di->dev, "End Gauge RESET\n");
    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_reset);

int bq275x0_battery_sealed(void)
{
    int ret;

    ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_SEALED,
                                            NULL,
                                            0
                                            );
    if (ret) {
        dev_err(di->dev, "error sealed\n");
        return ret;
    }

    dev_info(di->dev, "End Gauge SEALED\n");
    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_sealed);

static int bq275x0_battery_manufacturer_info(char* buf, u8 offset)
{
    int ret = 0;
    
    ret = bq275x0_battery_access_data_flash(57,
                                            offset,
                                            (u8*)buf,
                                            32,
                                            false
                                            );
    if (ret) {
        dev_err(di->dev, "error reading operation_configuration\n");
    }
    
    return ret;
}

int bq275x0_battery_enter_rom_mode(void)
{
    int ret = 0;

    ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_ROM_MODE,
                                            NULL,
                                            0
                                            );
    if (ret) {
        dev_err(di->dev, "failed to place bq275x0 into ROM MODE\n");
    }

    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_enter_rom_mode);

#if 0
static int bq275x0_battery_application_status(void)
{
    int ret;
    u16 application_status;

    ret = bq275x0_read(di,
                        BQ275X0_EXT_CMD_APPSTAT,
                        (u8*)&application_status,
                        sizeof(application_status)
                        );
    if (ret) {
        dev_err(di->dev, "error reading application status\n");
        return ret;
    }
    
    return application_status;
}

static int bq275x0_battery_operation_configuration(void)
{
    int ret;
    u16 operation_configuration;
    
    ret = bq275x0_battery_access_data_flash(64,
                                            0,
                                            (u8*)&operation_configuration,
                                            sizeof(operation_configuration),
                                            false
                                            );
    if (ret) {
        dev_err(di->dev, "error reading operation_configuration\n");
        return ret;
    }
    
    return operation_configuration;
}
#endif

static int bq275x0_battery_control_status(void)
{
    int ret;
    u16 control_status = 0;

    ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_CONTROL_STATUS,
                                            (u8*)&control_status,
                                            sizeof(u16)
                                            );
    if (ret) {
        dev_err(di->dev, "error reading control status\n");
        return ret;
    }

    return control_status;
};

static int bq275x0_battery_snooze_mode(bool set)
{
    int ret;
    u16 control_status = 0;

    if (set)
        ret = bq275x0_battery_control_command(BQ275X0_CNTLDATA_SET_SLEEP_PLUS,
                                                NULL,
                                                0
                                                );
    else
        ret = bq275x0_battery_control_command( BQ275X0_CNTLDATA_CLEAR_SLEEP_PLUS,
                                                NULL,
                                                0
                                                );
    if (ret) {
        dev_err(di->dev, "error reading firmware version\n");
        return ret;
    }
    
    control_status = bq275x0_battery_control_status();
    if (set) {
        return !(control_status & (1 << BQ275X0_CNTLSTATUS_SNOOZE));
    } else {
        return control_status & (1 << BQ275X0_CNTLSTATUS_SNOOZE);
    }

    return ret;
}

static int bq275x0_battery_charge_inhibit_subclass(void)
{
    int ret;
    //s16 temp_hys;
    
    ret = bq275x0_battery_access_data_flash(32,
                                            0,
                                            (u8*)&di->chg_inh_temp_l,
                                            sizeof(s16),
                                            false
                                            );
    if (ret) {;
        dev_err(di->dev, "error reading charge inhibit temp low\n");
        return ret;
    }
    
    ret = bq275x0_battery_access_data_flash(32,
                                            2,
                                            (u8*)&di->chg_inh_temp_h,
                                            sizeof(s16),
                                            false
                                            );
    if (ret) {
        dev_err(di->dev, "error reading charge inhibit temp high\n");
        return ret;
    }
    
    /*ret = bq275x0_battery_access_data_flash(32,
                                            4,
                                            (u8*)&temp_hys,
                                            sizeof(temp_hys),
                                            false
                                            );
    if (ret) {
        dev_err(di->dev, "error reading operation_configuration\n");
        return ret;
    }*/

    bq275x0_endian_adjust((u8*)&di->chg_inh_temp_l, sizeof(u16));
    bq275x0_endian_adjust((u8*)&di->chg_inh_temp_h, sizeof(u16));
    //bq275x0_endian_adjust((u8*)&temp_hys, sizeof(temp_hys));
    
    return 0;
}

#define to_bq275x0_device_info(x) container_of((x), \
                struct bq275x0_device_info, bat);

static int bq275x0_battery_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    struct bq275x0_device_info *di = to_bq275x0_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = di->voltage_uV * 1000;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = di->present;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = di->current_uA;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = di->charge_rsoc;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = di->temp_C;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_STATUS:
        val->intval =  msm_batt_get_batt_status();
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = di->health;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static void bq275x0_powersupply_init(struct bq275x0_device_info *di)
{
    di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
    di->bat.properties = bq275x0_battery_props;
    di->bat.num_properties = ARRAY_SIZE(bq275x0_battery_props);
    di->bat.get_property = bq275x0_battery_get_property;
    di->bat.external_power_changed = NULL;
}

static int bq275x0_battery_config_gpio(struct bq275x0_device_info *di)
{
#ifndef CONFIG_FIH_FTM_BATTERY_CHARGING
    struct pm8058_gpio gpio_configuration = {
        .direction      = PM_GPIO_DIR_IN,
        .pull           = PM_GPIO_PULL_NO,
        .vin_sel        = 2,
        .out_strength   = PM_GPIO_STRENGTH_NO,
        .function       = PM_GPIO_FUNC_NORMAL,
        .inv_int_pol    = 0,
    };
    int rc = 0;

    printk(KERN_INFO "%s\n", __func__);

    rc = pm8058_gpio_config(di->pmic_BATGD, &gpio_configuration);
    if (rc < 0) {
        dev_err(&di->client->dev, "%s: gpio %d configured failed\n", __func__, di->pmic_BATGD);
        return rc;
    } else {
        rc = gpio_request(di->pmic_BATGD + NR_GPIO_IRQS, "BAT_GD");
        if (rc < 0) {
            dev_err(&di->client->dev, "%s: gpio %d requested failed\n", __func__, di->pmic_BATGD + NR_GPIO_IRQS);
            return rc;
        } else {
            rc = gpio_direction_input(di->pmic_BATGD + NR_GPIO_IRQS);
            if (rc < 0) {
                dev_err(&di->client->dev, "%s: set gpio %d direction failed\n", __func__, di->pmic_BATGD + NR_GPIO_IRQS);
                return rc;
            }
        }
    }
    
    rc = pm8058_gpio_config(di->pmic_BATLOW, &gpio_configuration);
    if (rc < 0) {
        dev_err(&di->client->dev, "%s: gpio %d configured failed\n", __func__, di->pmic_BATLOW);
        return rc;
    } else {
        rc = gpio_request(di->pmic_BATLOW + NR_GPIO_IRQS, "BAT_LOW");
        if (rc < 0) {
            dev_err(&di->client->dev, "%s: gpio %d requested failed\n", __func__, di->pmic_BATLOW + NR_GPIO_IRQS);
            return rc;
        } else {
            rc = gpio_direction_input(di->pmic_BATLOW + NR_GPIO_IRQS);
            if (rc < 0) {
                dev_err(&di->client->dev, "%s: set gpio %d direction failed\n", __func__, di->pmic_BATLOW + NR_GPIO_IRQS);
                return rc;
            }
        }
    }
    
    return rc;
#else  /*CONFIG_FIH_FTM_BATTERY_CHARGING*/
    return 0;
#endif  
}

static void bq275x0_battery_release_gpio(struct bq275x0_device_info *di)
{
    gpio_free(di->pmic_BATGD + NR_GPIO_IRQS);
    gpio_free(di->pmic_BATLOW + NR_GPIO_IRQS);
}

static void bq275x0_battery_BATGD(struct work_struct *work)
{
    struct bq275x0_device_info *di  = container_of(work, struct bq275x0_device_info, bq275x0_BATGD);
    u16 flags = bq275x0_battery_flags();
    bool current_chg_inh;
    bool gpio_val               = (bool)gpio_get_value(di->pmic_BATGD);
    
    dev_info(&di->client->dev, "%s: BAT_GD  %d, flags  0x%02x\n", __func__, gpio_val, flags);
    
    di->present = flags & (0x1 << BQ275X0_FLAGS_BAT_DET);       /* Battery Detection Flag */
    if (!di->present) {
        dev_err(&di->client->dev, "%s: the battery is removed\n", __func__);
        power_supply_changed(&di->bat);
    }
        
    current_chg_inh = flags & (0x1 << BQ275X0_FLAGS_CHG_INH);   /* Charge Inhibit Flag */
    if (current_chg_inh != di->chg_inh) {
        msm_batt_OT_protection();
        di->chg_inh = current_chg_inh;
    }
}

static void bq275x0_battery_BATLOW(struct work_struct *work)
{
    struct bq275x0_device_info *di  = container_of(work, struct bq275x0_device_info, bq275x0_BATLOW);
    bool gpio_val                   = (bool)gpio_get_value(di->pmic_BATLOW);
    
    dev_info(&di->client->dev, "%s: BAT_LOW  %d\n", __func__, gpio_val);

    if (gpio_val) {
    } else {
    }
}

static irqreturn_t bq275x0_battery_irqhandler(int irq, void *dev_id)
{
    struct bq275x0_device_info *di = (struct bq275x0_device_info *) dev_id;

    if (di->dev_ready) {
        if (PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, di->pmic_BATGD) == irq) {    
            schedule_work(&di->bq275x0_BATGD);
        } else if (PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, di->pmic_BATLOW) == irq) {    
            schedule_work(&di->bq275x0_BATLOW);
        }
    }
    
    return IRQ_HANDLED;
}

static int bq275x0_battery_irqsetup(struct bq275x0_device_info *di)
{
    int rc;
    
    rc = request_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, di->pmic_BATGD), &bq275x0_battery_irqhandler,
                 (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), 
                 "bq275x0-BATGD", di);
    if (rc < 0) {
        dev_err(&di->client->dev,
               "Could not register for bq275x0_battery interrupt %d"
               "(rc = %d)\n", di->pmic_BATGD, rc);
        rc = -EIO;
    }
    
    rc = request_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, di->pmic_BATLOW), &bq275x0_battery_irqhandler,
                 (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), 
                 "bq275x0-BATLOW", di);
    if (rc < 0) {
        dev_err(&di->client->dev,
               "Could not register for bq275x0_battery interrupt %d"
               "(rc = %d)\n", di->pmic_BATLOW, rc);
        rc = -EIO;
    }
        
    return rc;
}

static void bq275x0_battery_free_irq(struct bq275x0_device_info *di)
{
    free_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, di->pmic_BATGD), di);
    free_irq(PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, di->pmic_BATLOW), di);
}

static void bq275x0_battery_update_batt_status(void)
{
    int flags = bq275x0_battery_flags();
    
    if (flags & (0x1 << BQ275X0_FLAGS_FC)) /* Fully Charged Flag */
        msm_batt_notify_battery_full();
    
    di->voltage_uV      = bq275x0_battery_voltage();
    di->charge_rsoc     = bq275x0_battery_soc();
    di->current_uA      = bq275x0_battery_current();
    di->temp_C          = bq275x0_battery_temperature();
    di->health          = bq275x0_battery_health();
}

static void bq275x0_battery_update(struct work_struct *work)
{
    bq275x0_battery_update_batt_status();
    power_supply_changed(&di->bat);
    mod_timer(&di->polling_timer,
            jiffies + msecs_to_jiffies(di->polling_interval));
}

#ifdef CONFIG_FIH_FTM_BATTERY_CHARGING
enum {
    FTM_CHARGING_SET_CHARGING_STATUS,
    FTM_CHARGING_BACKUPBATTERY_ADC,  
};

struct manufacturer_info g_manufacturer_info;
static u16 g_device_type = 0;
static int g_capacity   = 0;
static int g_voltage    = 0;
static int g_temp       = 0;
static int g_family_code= 0;
static enum _battery_info_type_ g_current_cmd = 0;
static int g_smem_error = -1;
static int g_bb_voltage = -1;
static int g_charging_on= 0;

extern int proc_comm_config_coin_cell(int vset, int *voltage/*, int rset*/);
extern int proc_comm_config_chg_current(bool on, int curr);

static ssize_t bq275x0_battery_ftm_battery_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    int cmd     = 0;
    
    sscanf(buf, "%d", &cmd);
    dev_info(dev, "%s: COMMAND: %d\n", __func__, cmd);

    g_current_cmd = cmd;
    switch (cmd) {
    case BATT_CAPACITY_INFO:
        g_capacity = bq275x0_battery_soc();
        break;
    case BATT_VOLTAGE_INFO:
        g_voltage = bq275x0_battery_voltage();
        break;
    case BATT_TEMPERATURE_INFO:
        g_temp = bq275x0_battery_temperature();
        break;
    case BATT_FAMILY_CODE:
        g_family_code = bq275x0_battery_fw_version();
        break;
    case BATT_GET_MANUFACTURER_INFO:
        bq275x0_battery_manufacturer_info((u8*)&g_manufacturer_info, 0);          /* block A */
        bq275x0_endian_adjust(g_manufacturer_info.date, 4);
        break;
    case BATT_GET_DEVICE_TYPE:
        g_device_type = bq275x0_battery_device_type();
        break;
    default:
        break;
    };

    return count;
}

static ssize_t bq275x0_battery_ftm_battery_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    char project_name[7];

    switch (g_current_cmd) {
    case BATT_CAPACITY_INFO:         // 0x06 RARC - Remaining Active Relative Capacity
        return sprintf(buf, "%d\n", g_capacity);
    case BATT_VOLTAGE_INFO:         // 0x0c Voltage MSB
        return sprintf(buf, "%d\n", g_voltage);
    case BATT_TEMPERATURE_INFO:      // 0x0a Temperature MSB
        return sprintf(buf, "%d\n", g_temp);
    case BATT_FAMILY_CODE:           // 0x33
        return sprintf(buf, "%x\n", g_family_code);
    case BATT_GET_MANUFACTURER_INFO:
        strncpy(project_name, g_manufacturer_info.prj_name, 6);
        project_name[6] = '\0';
        return sprintf(buf, "%02x-%s-%08x-%02x%02x-%02x\n",
                            g_manufacturer_info.dfi_ver,
                            project_name,
                            *((u32*)g_manufacturer_info.date),
                            g_manufacturer_info.other[0],
                            g_manufacturer_info.other[1],
                            g_manufacturer_info.other[2]
                        );
    case BATT_GET_DEVICE_TYPE:
        return sprintf(buf, "%x\n", g_device_type);
    default:
        return 0;
    };
    
    return 0;
}

static ssize_t bq275x0_battery_ftm_charging_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    int cmd     = 0;
    int data[3] = {0, 0, 0};
    
    sscanf(buf, "%d %d %d %d", &cmd, &data[0], &data[1], &data[2]);
    dev_info(dev, "%s: COMMAND: %d data1 = %d data2 = %d\n", __func__, cmd, data[0], data[1]);
    
    switch (cmd) {
    case FTM_CHARGING_SET_CHARGING_STATUS:
        if (data[2] == 1) {
            g_charging_on = data[0];
        } 
        g_smem_error = proc_comm_config_chg_current((g_charging_on ? true : false), data[1]);
        break;
    case FTM_CHARGING_BACKUPBATTERY_ADC:
        g_smem_error = proc_comm_config_coin_cell(2, &g_bb_voltage);
        break;
    }
    
    return count;
}

static ssize_t bq275x0_battery_ftm_charging_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d %d\n", g_bb_voltage, g_smem_error);
}

static DEVICE_ATTR(ftm_battery, 0644, bq275x0_battery_ftm_battery_show, bq275x0_battery_ftm_battery_store);
static DEVICE_ATTR(ftm_charging, 0644, bq275x0_battery_ftm_charging_show, bq275x0_battery_ftm_charging_store);
#endif

static void bq275x0_battery_polling_timer_func(unsigned long unused)
{
    schedule_work(&di->bq275x0_update);
}

#ifdef CONFIG_PM
static int bq275x0_battery_suspend(struct device *dev)
{
    return 0;
}

static int bq275x0_battery_resume(struct device *dev)
{
    bq275x0_battery_update_batt_status();
    power_supply_changed(&di->bat);
    return 0;
}

static struct dev_pm_ops bq275x0_battery_pm = {
    .suspend = bq275x0_battery_suspend,
    .resume = bq275x0_battery_resume,
};
#endif

static int bq275x0_battery_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    struct bq275x0_platform_data *bq275x0_pd = client->dev.platform_data;
    struct manufacturer_info gauge_info;
    char project_name[7];
    char *name;
    int num;
    int retval = 0;

    /* Get new ID for the new battery device */
    retval = idr_pre_get(&battery_id, GFP_KERNEL);
    if (retval == 0)
        return -ENOMEM;
    mutex_lock(&battery_mutex);
    retval = idr_get_new(&battery_id, client, &num);
    mutex_unlock(&battery_mutex);
    if (retval < 0)
        return retval;

    name = kasprintf(GFP_KERNEL, "bq275x0-%d", num);
    if (!name) {
        dev_err(&client->dev, "failed to allocate device name\n");
        retval = -ENOMEM;
        goto batt_failed_1;
    }

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di) {
        dev_err(&client->dev, "failed to allocate device info data\n");
        retval = -ENOMEM;
        goto batt_failed_2;
    }
    di->id              = num;
    di->dev             = &client->dev;
    di->bat.name        = name;
    di->client          = client;
    di->pmic_BATGD      = bq275x0_pd->pmic_BATGD;
    di->pmic_BATLOW     = bq275x0_pd->pmic_BATLOW;
    di->dev_ready       = false;
    di->chg_inh         = false;
    di->present         = 1;
    di->polling_interval= BATTERY_POLLING_TIMER;
    
    i2c_set_clientdata(client, di);
    
    retval = bq275x0_battery_config_gpio(di);
    if (retval) {
        dev_err(&client->dev, "failed to config gauge GPIOs\n");
        goto batt_failed_3;        
    }
    
    retval = bq275x0_battery_irqsetup(di);
    if (retval) {
        dev_err(&client->dev, "failed to config gauge IRQs\n");
        goto batt_failed_4;
    }    

    bq275x0_powersupply_init(di);

    retval = power_supply_register(&client->dev, &di->bat);
    if (retval) {
        dev_err(&client->dev, "failed to register battery\n");
        goto batt_failed_5;
    }
    
    INIT_WORK(&di->bq275x0_BATGD, bq275x0_battery_BATGD);
    INIT_WORK(&di->bq275x0_BATLOW, bq275x0_battery_BATLOW);
    INIT_WORK(&di->bq275x0_update, bq275x0_battery_update);

    di->dev_ready = true;
    
    setup_timer(&di->polling_timer,
                bq275x0_battery_polling_timer_func, 0);
    mod_timer(&di->polling_timer,
                jiffies + msecs_to_jiffies(di->polling_interval));    

    if (bq275x0_battery_snooze_mode(true)) {
        dev_info(&client->dev, "Set SNOOZE mode failed\n");
    }
    
    bq275x0_battery_update_batt_status();
    bq275x0_battery_charge_inhibit_subclass();
    bq275x0_battery_manufacturer_info((u8*)&gauge_info, 0);
    bq275x0_endian_adjust(gauge_info.date, 4);
    strncpy(project_name, gauge_info.prj_name, 6);
    project_name[6] = '\0';
    dev_info(&client->dev, "device type %x, fw ver. %x, dfi info %02x-%s-%08x-%02x%02x-%02x, chg inhibit low %d, chg inhibit high %d\n",
                            bq275x0_battery_device_type(),
                            bq275x0_battery_fw_version(),
                            gauge_info.dfi_ver,
                            project_name,
                            *((u32*)gauge_info.date),
                            gauge_info.other[0],
                            gauge_info.other[1],
                            gauge_info.other[2],
                            di->chg_inh_temp_l,
                            di->chg_inh_temp_h);

    dev_info(&client->dev, "%s finished\n", __func__);
    
#ifdef CONFIG_FIH_FTM_BATTERY_CHARGING
    retval = device_create_file(&client->dev, &dev_attr_ftm_battery);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_ftm_battery failed\n", __func__);
    }
    retval = device_create_file(&client->dev, &dev_attr_ftm_charging);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_ftm_charging failed\n", __func__);
    }
#endif

    return 0;

batt_failed_5:
    bq275x0_battery_free_irq(di);
batt_failed_4:
    bq275x0_battery_release_gpio(di);
batt_failed_3:
    kfree(di);
batt_failed_2:
    kfree(name);
batt_failed_1:
    mutex_lock(&battery_mutex);
    idr_remove(&battery_id, num);
    mutex_unlock(&battery_mutex);

    return retval;
}

static int bq275x0_battery_remove(struct i2c_client *client)
{
    struct bq275x0_device_info *di = i2c_get_clientdata(client);
    
    if (bq275x0_battery_snooze_mode(false)) {
        dev_info(&client->dev, "Clear SNOOZE mode failed\n");
    }
    
    di->dev_ready = false;
    
    bq275x0_battery_free_irq(di);
    bq275x0_battery_release_gpio(di);

    flush_work(&di->bq275x0_BATGD);
    flush_work(&di->bq275x0_BATLOW);

    power_supply_unregister(&di->bat);

    kfree(di->bat.name);

    mutex_lock(&battery_mutex);
    idr_remove(&battery_id, di->id);
    mutex_unlock(&battery_mutex);
    
    wake_lock_destroy(&di->bq275x0_wakelock);

    kfree(di);

    return 0;
}

/*
 * Module stuff
 */
static const struct i2c_device_id bq275x0_id[] = {
    { "bq275x0-battery", 0 },
    {},
};

static struct i2c_driver bq275x0_battery_driver = {
    .driver = {
        .name   = "bq275x0-battery",
#ifdef CONFIG_PM
        .pm     = &bq275x0_battery_pm,
#endif
    },
    .probe = bq275x0_battery_probe,
    .remove = bq275x0_battery_remove,
    .id_table = bq275x0_id,
};

static int __init bq275x0_battery_init(void)
{
    int ret;

    ret = i2c_add_driver(&bq275x0_battery_driver);
    if (ret)
        printk(KERN_ERR "Unable to register bq275x0 driver\n");

    return ret;
}
module_init(bq275x0_battery_init);

static void __exit bq275x0_battery_exit(void)
{
    i2c_del_driver(&bq275x0_battery_driver);
}
module_exit(bq275x0_battery_exit);

MODULE_AUTHOR("Audi PC Huang <AudiPCHuang@fihtdc.com>");
MODULE_DESCRIPTION("bq275x0 battery monitor driver");
MODULE_LICENSE("GPL");
