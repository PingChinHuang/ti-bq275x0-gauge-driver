/*
 * BQ275x0 dfi programming driver
 */
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#define BQ275X0_DFI_PATH        "/ftm/bq275x0.dfi"
#define ERASE_RETRIES           3

static struct i2c_client *bq275x0_client;
static u8 dfi[1024];
static u8 ifrow_data[2][96];
/* Firmware Backup*/
#if 0
static u8 ifrow_data_backup[2][96] = {
    {
        0xEA, 0xFF, 0x33, 0x5C, 0xF5, 0x33, 0x03, 0xF6,
        0x33, 0x26, 0xFE, 0x33, 0x2E, 0xFE, 0x33, 0x54,
        0x54, 0x15, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0x3F,
        0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF,
        0x3F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0x3F, 0xFF,
        0xFF, 0x3F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0x3F,
        0x30, 0x01, 0x32, 0x58, 0xCB, 0x33, 0x00, 0x00,
        0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
        0xAA, 0x0E, 0xFB, 0xA7, 0x0E, 0xFF, 0xA6, 0x0E,
        0xFF, 0xA1, 0x0E, 0xFF, 0xA0, 0x0E, 0xFE, 0xA3,
        0x0E, 0x35, 0xA2, 0x0E, 0xD2, 0xFF, 0x3A, 0xFF,
        0xA1, 0x0E, 0x6B, 0xA0, 0x0E, 0xFF, 0xA3, 0x0E
    },
    {
        0x6B, 0xA2, 0x0E, 0xBC, 0xA5, 0x0E, 0xFF, 0xA4,
        0x0E, 0xCA, 0xFF, 0x3A, 0xFE, 0xA1, 0x0E, 0x35,
        0xA0, 0x0E, 0xFE, 0xA3, 0x0E, 0x35, 0xA2, 0x0E,
        0xCA, 0xFF, 0x3A, 0x51, 0xFF, 0x3A, 0xD6, 0xFF,
        0x33, 0xFF, 0xAF, 0x0E, 0x01, 0x4F, 0x03, 0x3F,
        0x11, 0x0C, 0xCD, 0xFF, 0x30, 0xD4, 0xFF, 0x35,
        0x2F, 0x10, 0x0C, 0xD4, 0xFF, 0x35, 0xFF, 0xFF,
        0x23, 0x01, 0xAF, 0x14, 0x01, 0x4F, 0x03, 0x3F,
        0x11, 0x0C, 0xC5, 0xFF, 0x30, 0xCC, 0xFF, 0x35,
        0x2F, 0x10, 0x0C, 0xCC, 0xFF, 0x35, 0xFF, 0xFF,
        0x23, 0xFF, 0xDF, 0x0B, 0xC4, 0xFF, 0x33, 0xE3,
        0xAF, 0x04, 0xE2, 0xBF, 0x04, 0xBB, 0xFF, 0x36
    }
};
#endif
static u16 dfi_checksum = 0;
static int g_error_type = 0;
static int g_show_error = 0;
static u16 g_fw_ver     = 0;
static u16 g_dev_type   = 0;
static bool g_rom_mode  = false;    

extern int bq275x0_battery_fw_version(void);
extern int bq275x0_battery_device_type(void);
extern int bq275x0_battery_enter_rom_mode(void);
extern int bq275x0_battery_IT_enable(void);
extern int bq275x0_battery_reset(void);
extern int bq275x0_battery_sealed(void);

enum {
    ERR_SUCCESSFUL,
    ERR_READ_DFI,
    ERR_ENTER_ROM_MODE,
    ERR_READ_IF,
    ERR_ERASE_IF,
    ERR_MASS_ERASE,
    ERR_PROGRAM_DFI,
    ERR_CHECKSUM,
    ERR_PROGRAM_IF,
    ERR_IT_ENABLE,
    ERR_RESET,
    ERR_SEALED,
	ERR_DEVICE_TYPE_FW_VER,
    ERR_UNKNOWN_CMD,
};

static int bq275x0_RomMode_read(u8 cmd, u8 *data, int length)
{
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = bq275x0_client->addr,
            .flags  = 0,
            .buf    = (void *)&cmd,
            .len    = 1
        },
        [1] = {
            .addr   = bq275x0_client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)data,
            .len    = length
        }
    };

    return (i2c_transfer(bq275x0_client->adapter, msgs, 2) < 0) ? -1 : 0;    
}

static int bq275x0_RomMode_write(u8 *cmd, int length)
{
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = bq275x0_client->addr,
            .flags  = 0,
            .buf    = (void *)cmd,
            .len    = length
        },
    };

    return (i2c_transfer(bq275x0_client->adapter, msgs, 1) < 0) ? -1 : 0;    
}

static int bq275x0_RomMode_read_flash_image(void)
{
    mm_segment_t oldfs; 
    struct file *fp = NULL;
    
    oldfs = get_fs(); 
    set_fs(KERNEL_DS); 

    if (g_dev_type == 0x500 && g_fw_ver == 0x130) {
	    fp = filp_open(BQ275X0_DFI_PATH, O_RDONLY, 0);

	    if (fp == NULL) {
	        set_fs(oldfs);
	        dev_err(&bq275x0_client->dev, "Reading .dfi file was failed\n");
	        return -ERR_READ_DFI;
	    }

	    fp->f_op->read(fp, dfi, sizeof(dfi), &fp->f_pos);
	    filp_close(fp, NULL);
    } else {
		set_fs(oldfs);
		dev_err(&bq275x0_client->dev, "device type or fw version do not match\n");
		return -ERR_DEVICE_TYPE_FW_VER;
	}
    set_fs(oldfs);
    
    return 0;
}

static int bq275x0_RomMode_read_first_two_rows(void)
{
    u8 irow = 0;
    u8 cmd[4];

    for (irow = 0; irow < 2; irow++) {
        cmd[0] = 0x00;
        cmd[1] = 0x00;          /* 0x00 */
        cmd[2] = 0x00 + irow;   /* 0x01 */
        cmd[3] = 0x00;          /* 0x02 */
        if (bq275x0_RomMode_write(cmd, sizeof(cmd)))
            return -1;
        cmd[0] = 0x64;
        cmd[1] = 0x00 + irow;   /* 0x64 */
        cmd[2] = 0x00;          /* 0x65 */
        if (bq275x0_RomMode_write(cmd, 3))
            return -1;
        
        if (bq275x0_RomMode_read(0x04, ifrow_data[irow], sizeof(ifrow_data[irow])))
            return -1;
        mdelay(20);
    }
    
    return 0;
}

static int bq275x0_RomMode_erase_first_two_rows(void)
{
    u8 cmd[3];
    u8 buf;
    u8 retries = 0;
    
    do {
        retries++;
        
        cmd[0] = 0x00;
        cmd[1] = 0x03;
        bq275x0_RomMode_write(cmd, 2);
        cmd[0] = 0x64;
        cmd[1] = 0x03;  /* 0x64 */
        cmd[2] = 0x00;  /* 0x65 */
        bq275x0_RomMode_write(cmd, sizeof(cmd));
        mdelay(20);
        
        bq275x0_RomMode_read(0x66, &buf, sizeof(buf));
        if (buf == 0x00)
            return 0;
            
        if (retries > ERASE_RETRIES)
            break;
    } while (1);
    
    return -1;
}

static int bq275x0_RomMode_mass_erase(void)
{
    u8 cmd[3];
    u8 buf;
    u8 retries = 0;

    do {
        retries++;
        
        cmd[0] = 0x00;
        cmd[1] = 0x0C;
        bq275x0_RomMode_write(cmd, 2);
        cmd[0] = 0x04;
        cmd[1] = 0x83;  /* 0x04 */
        cmd[2] = 0xDE;  /* 0x05 */
        bq275x0_RomMode_write(cmd, sizeof(cmd));
        cmd[0] = 0x64;
        cmd[1] = 0x6D;  /* 0x64 */
        cmd[2] = 0x01;  /* 0x65 */
        bq275x0_RomMode_write(cmd, sizeof(cmd));
        mdelay(200);
    
        bq275x0_RomMode_read(0x66, &buf, sizeof(buf));
        if (buf == 0x00)
            return 0;
            
        if (retries > ERASE_RETRIES)
            break;
    } while (1);
    
    return -1;
}

static int bq275x0_RomMode_program_row(void)
{
    int irow = 0;
    int sum = 0;
    int i = 0;
    u8 row_data[33];
    u8 cmd[3];
    u8 buf;
    u16* checksum = (u16*)(cmd + 1);
    
    row_data[0] = 0x04;
    for (irow = 0; irow < sizeof(dfi) / 32; irow++) {
        memcpy(&row_data[1], &dfi[irow * 32], 32); 
        
        cmd[0] = 0x00;
        cmd[1] = 0x0A;  /* 0x00 */
        cmd[2] = irow;  /* 0x01 */
        bq275x0_RomMode_write(cmd, sizeof(cmd));
        
        bq275x0_RomMode_write(row_data, sizeof(row_data));
        
        sum = 0;
        for (i = 1; i < 33; i++)
            sum += row_data[i];
        *checksum = (0x0A + irow + sum) % 0x10000;
        dfi_checksum = (dfi_checksum + sum) % 0x10000;
        
        cmd[0] = 0x64;
        bq275x0_RomMode_write(cmd, sizeof(cmd));        
        mdelay(2);
        
        bq275x0_RomMode_read(0x66, &buf, sizeof(buf));
        if (buf != 0x00) {
            dev_err(&bq275x0_client->dev, "Program row %d was failed\n", irow);
            return -1;
        }
    }
    
    return 0;
}

static int bq275x0_RomMode_setup_data_flash_checksum(void)
{
    u8 cmd[3];
    u16 dfi_checksum_rb = 0;
    
    cmd[0] = 0x00;
    cmd[1] = 0x08;
    bq275x0_RomMode_write(cmd, 2);
    cmd[0] = 0x64;
    cmd[1] = 0x08;  /* 0x64 */
    cmd[2] = 0x00;  /* 0x65 */
    bq275x0_RomMode_write(cmd, sizeof(cmd));
    mdelay(20);
    
    bq275x0_RomMode_read(0x04, (u8*)&dfi_checksum_rb, sizeof(dfi_checksum_rb));
    if (dfi_checksum != dfi_checksum_rb)
        return -1;
        
    return 0;
}

static int bq275x0_RomMode_read_erase_if(void)
{
    int ret = 0;
    
    if (bq275x0_battery_enter_rom_mode()) {
        dev_err(&bq275x0_client->dev, "Entering rom mode was failed\n");
        ret = -ERR_ENTER_ROM_MODE; /* Rom Mode is locked, or IC is damaged */
    }
    mdelay(2); /* Gauge need time to enter ROM mode. */
        
    /* Try to read IF rows. If reading failed and entering rom mode failed 
     * , this IC must be damaged. Send ERR_ENTER_ROM_MODE to identify this
     * situation.
     * */
    if (bq275x0_RomMode_read_first_two_rows()) {
        dev_err(&bq275x0_client->dev, "Reading IF first two rows was failed\n");
        if (ret)
            return ret;
            
        return -ERR_READ_IF;
    }
    
    if (bq275x0_RomMode_erase_first_two_rows()) {
        dev_err(&bq275x0_client->dev, "Erasing IF first two rows was failed\n");
        if (ret)
            return ret;
            
        return -ERR_ERASE_IF;
    }
    
    dev_info(&bq275x0_client->dev, "Start to Writing Image\n");
    return 0;
}

static int bq275x0_RomMode_writing_image(void)
{
    int retries = 0;
    int error_type = 0;
    bool error_occurred = false;
    
    do {
        error_occurred = false;
        dfi_checksum = 0;
        retries++;
        
        if (bq275x0_RomMode_mass_erase()) {
            dev_err(&bq275x0_client->dev, "Mass Erase procedure was failed\n");
            error_type = -ERR_MASS_ERASE;
            error_occurred = true;
            continue;
        }
        
        if (bq275x0_RomMode_program_row()) {
            error_type = -ERR_PROGRAM_DFI;
            error_occurred = true;
            continue;
        }
            
        if (bq275x0_RomMode_setup_data_flash_checksum()) {
            dev_err(&bq275x0_client->dev, "Setup data flash checksum was failed\n");
            error_type = -ERR_CHECKSUM;
            error_occurred = true;
        }
        
        if (retries > ERASE_RETRIES) {
            dev_err(&bq275x0_client->dev, "Writing image was failed\n");      
            return error_type;
        }
    } while (error_occurred);

    dev_info(&bq275x0_client->dev, "End of Writing Image\n");
    return 0;
}

static int bq275x0_RomMode_reprogram_if(void)
{
    int i = 0;
    int sum = 0;
    int irow = 1;
    u8 buf;
    u8 cmd[4];
    u8 row_data[97];
    u16* checksum = (u16*)(cmd + 1);
    u8 retries = 0;

    row_data[0] = 0x04;
    for (irow = 1; irow >= 0; irow--) {
        memcpy(&row_data[1], ifrow_data[irow], 96);
        retries = 0;
        
        do {
            retries++;
            
            cmd[0] = 0x00;
            cmd[1] = 0x02;          /* 0x00 */
            cmd[2] = 0x00 + irow;   /* 0x01 */
            cmd[3] = 0x00;          /* 0x02 */
            bq275x0_RomMode_write(cmd, sizeof(cmd));
            
            bq275x0_RomMode_write(row_data, sizeof(row_data));
            
            sum = 0;
            for (i = 1; i < 97; i ++)
                sum += row_data[i];
            *checksum = (0x02 + irow + sum) % 0x10000;
            
            cmd[0] = 0x64;
            bq275x0_RomMode_write(cmd, 3);
            mdelay(20);
            
            bq275x0_RomMode_read(0x66, &buf, sizeof(buf));
            if (retries > ERASE_RETRIES) {
                dev_err(&bq275x0_client->dev, "Program IF row %d was failed\n", irow);
                return -ERR_PROGRAM_IF;
            }
        } while (buf != 0x00);
    }
    
    cmd[0] = 0x00;
    cmd[1] = 0x0F;
    bq275x0_RomMode_write(cmd, 2);
    
    cmd[0] = 0x64;
    cmd[1] = 0x0F;  /* 0x64 */
    cmd[2] = 0x00;  /* 0x65 */
    bq275x0_RomMode_write(cmd, 3);

    mdelay(2); /* Gauge need time to exit ROM mode. */
    dev_info(&bq275x0_client->dev, "End Program IF Row\n");
    return 0;
}

enum {
    PROGRAM_READ_DFI,
    PROGRAM_READ_ERASE_IF,
    PROGRAM_WRITING_DFI,
    PROGRAM_PROGRAM_IF,
    PROGRAM_IT_ENABLE,
    PROGRAM_RESET,
    PROGRAM_SEALED,
};

static ssize_t bq275x0_RomMode_programming_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{  
    int step    = 0;
    int cmd     = 0;
    int ret     = 0;
    
    /*
     * cmd:     Assign first command.
     * step:    Assign how many commands we want to execute since first
     *          command.
     * g_show_error: When read this attribute,
     *          0. show DFI and IF contents.
     *          1. show error number.
     * 
     * If PROGRAM_WRITING_DFI command is failed, IF rows should not be 
     * reporgrammed. If IF rows are empty, the IC will always enter ROM 
     * Mode.
     * 
     * PROGRAM_RESET command is strongly suggested to execute after
     * PROGRAM_IT_ENABLE command is executed.
     * 
     * PROGRAM_SEALED command need other command to execute it. This way
     * is to prevent from entering SEALED mode unexpectedly.
     */
    sscanf(buf, "%d %d %d", &cmd, &step, &g_show_error);
    switch(cmd) {
    case PROGRAM_READ_DFI:      /* cmd = PROGRAM_READ_DFI, step 6 */
        if (step)
            step--;
        else
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
        if ((g_error_type = bq275x0_RomMode_read_flash_image()))
            break;
    case PROGRAM_READ_ERASE_IF: /* step 5 */
        if (step)
            step--;
        else
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
            
        if ((g_error_type = bq275x0_RomMode_read_erase_if()))
            break;
    case PROGRAM_WRITING_DFI:   /* step 4 */
        if (step > 0) {
            if (step != 1)
                step--;
        } else 
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);

        if ((g_error_type = bq275x0_RomMode_writing_image()))
            break;
    case PROGRAM_PROGRAM_IF:    /* step 3 */
        if (step)
            step--;
        else
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
            
        if ((g_error_type = bq275x0_RomMode_reprogram_if()))
            break;
    case PROGRAM_IT_ENABLE:     /* step 2 */
        if (step > 0) {
            if (step != 1)
                step--;
        } else
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
            
        if ((ret = bq275x0_battery_IT_enable())) {
            if (ret)
                g_error_type = -ERR_IT_ENABLE;
            break;
        }
    case PROGRAM_RESET:         /* step 1 */
        if (step)
            step--;
        else
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
            
        if ((ret = bq275x0_battery_reset())) {
            if (ret)
                g_error_type = -ERR_RESET;
            break;
        }
        break;
    case PROGRAM_SEALED:
        dev_info(&bq275x0_client->dev, "%s: +step %d\n", __func__, step);
        if (bq275x0_battery_sealed()) {
            g_error_type = -ERR_SEALED;
        }
        break;
    default:
        g_error_type = -ERR_UNKNOWN_CMD;
    };

    return count;
}

static ssize_t bq275x0_RomMode_programming_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int i = 0;
    int j = 0;
    int size = 0;
    
    if (g_show_error) {
        return sprintf(buf, "%d\n", g_error_type);
    } else {
        for (i = 0; i < sizeof(dfi); i++) {
            if ((i % 16 == 0) && (i != 0))
                size += sprintf(buf + size, "\n");
            size += sprintf(buf + size, "%02x ", dfi[i]);
        }
        
        size += sprintf(buf + size, "\n");
        
        for (i = 0; i < 2; i ++) {
            size += sprintf(buf + size, "\nIF ROW%d:\n", i);
            for (j = 0; j < 96; j++) {
                if ((j % 16 == 0) && (j != 0))
                    size += sprintf(buf + size, "\n");
                size += sprintf(buf + size, "%02x ", ifrow_data[i][j]);            
            }
            size += sprintf(buf + size, "\n");
        }
        
        size += sprintf(buf + size, "\n");
    }
    
    return size;
}

static DEVICE_ATTR(programming, 0644, bq275x0_RomMode_programming_show, bq275x0_RomMode_programming_store);

static int bq275x0_RomMode_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    int retval = 0;

    bq275x0_client = client;
    retval = device_create_file(&client->dev, &dev_attr_programming);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_test failed\n", __func__);
    }

    g_fw_ver        = bq275x0_battery_fw_version();
    g_dev_type		= bq275x0_battery_device_type();

    dev_info(&client->dev, "%s finished\n", __func__);

    return retval;
}

static int bq275x0_RomMode_remove(struct i2c_client *client)
{
    return 0;
}

/*
 * Module stuff
 */
static const struct i2c_device_id bq275x0_RomMode_id[] = {
    { "bq275x0-RomMode", 0 },
    {},
};

static struct i2c_driver bq275x0_RomMode_driver = {
    .driver = {
        .name = "bq275x0-RomMode",
    },
    .probe = bq275x0_RomMode_probe,
    .remove = bq275x0_RomMode_remove,
    .id_table = bq275x0_RomMode_id,
};

static int __init bq275x0_RomMode_init(void)
{
    int ret;

    ret = i2c_add_driver(&bq275x0_RomMode_driver);
    if (ret)
        printk(KERN_ERR "Unable to register bq275x0 RomMode driver\n");

    return ret;
}
module_init(bq275x0_RomMode_init);

static void __exit bq275x0_RomMode_exit(void)
{
    i2c_del_driver(&bq275x0_RomMode_driver);
}
module_exit(bq275x0_RomMode_exit);

MODULE_AUTHOR("Audi PC Huang <AudiPCHuang@fihtdc.com>");
MODULE_DESCRIPTION("bq275x0 dfi programming driver");
MODULE_LICENSE("GPL");
