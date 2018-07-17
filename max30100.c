#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>
#include "max30100.h"
#include <linux/sysfs.h>
#define DRV_VERSION "V2.0"

//i2c ADDRESS 0x57
static struct i2c_client *max30100_client;
static int i2c_max30100_read_len(struct i2c_client *client,unsigned char len,unsigned char *result_buf)
{
        int ret;
        struct i2c_msg msg[] = {
                {client->addr,I2C_M_RD,len,result_buf}
        };
        msg[0].scl_rate = 100*1000;
        ret = i2c_transfer(client->adapter,msg,ARRAY_SIZE(msg));
        if(ret < 0) {
                printk("i2c_transfer read len error\n");
                return -EINVAL;
        }
        return len;
}
static void setLEDs(unsigned char pw, unsigned char red, unsigned char ir) {
	unsigned char reg,ret;
	reg = i2c_smbus_read_byte_data(max30100_client,MAX30100_SPO2_CONFIG);
	reg = reg & 0xfc;
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_SPO2_CONFIG,reg | pw);
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_LED_CONFIG,(red<<4) | ir);
}
static void setSPO2(unsigned char sr) {
	unsigned char reg,ret;
	reg = i2c_smbus_read_byte_data(max30100_client,MAX30100_SPO2_CONFIG);
	reg &= 0xe3;
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_SPO2_CONFIG, reg | (sr<<2));
	reg = i2c_smbus_read_byte_data(max30100_client,MAX30100_MODE_CONFIG);
	reg &= 0xf8;
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_MODE_CONFIG, reg | 0x03 | 0x1 << 3);
}

static void setInterrupt(unsigned char intsrc) {
	unsigned char reg,ret;
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_INT_ENABLE,((intsrc + 1)<<4));
	reg = i2c_smbus_read_byte_data(max30100_client,MAX30100_INT_STATUS);
}
/*
static int getNumSamp() {
	unsigned char wr,rd,ret;
	wr = i2c_smbus_read_byte_data(max30100_client,MAX30100_FIFO_WR_PTR);
	rd = i2c_smbus_read_byte_data(max30100_client,MAX30100_FIFO_RD_PTR);
	return (( 16 + wrPtr - rdPtr ) % 16);
}
*/

static void readSensor(unsigned char *buf) {
	unsigned char reg,ret;
	int count;
	ret = i2c_smbus_read_byte_data(max30100_client,MAX30100_FIFO_RD_PTR);
	printk("address of MAX30100_FIFO_RD_PTR = %d\n",ret);
	ret = i2c_smbus_read_byte_data(max30100_client,MAX30100_FIFO_WR_PTR);
        printk("address of MAX30100_FIFO_WR_PTR = %d\n",ret);
	ret = i2c_smbus_read_byte_data(max30100_client,MAX30100_OVRFLOW_CTR);
        printk("address of MAX30100_OVRFLOW_CTR = %d\n",ret);
	//ret = i2c_smbus_read_block_data(max30100_client,MAX30100_FIFO_DATA,(s8 *)buf);
	i2c_smbus_write_byte(max30100_client,MAX30100_FIFO_DATA);
	i2c_max30100_read_len(max30100_client,4,buf);
	/*
	for(count = 0;count < 4;count++) {
		//buf[count] = i2c_smbus_read_byte(max30100_client);
		printk("buf[%d]=%d\n",count,*(buf+count));
	}
	*/
	
}
static void shutdown(void) {
	unsigned char reg,ret;
	reg = i2c_smbus_read_byte_data(max30100_client,MAX30100_MODE_CONFIG);
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_MODE_CONFIG,reg | 0x80);
}
static void reset(void) {
	unsigned char reg,ret;
        reg = i2c_smbus_read_byte_data(max30100_client,MAX30100_MODE_CONFIG);
        ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_MODE_CONFIG,reg | 0x40);
}
static void startup(void) {
	unsigned char reg,ret;
        reg = i2c_smbus_read_byte_data(max30100_client,MAX30100_MODE_CONFIG);
        ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_MODE_CONFIG,reg & 0x7f);
}
static unsigned char getRevID(void) {
	unsigned char ret;
	ret = i2c_smbus_read_byte_data(max30100_client,MAX30100_REV_ID);
	return ret;
}

static unsigned char getPartID(void) {
	unsigned char ret;
	ret = i2c_smbus_read_byte_data(max30100_client,MAX30100_PART_ID);
	return ret;
}
static int i = 0;
//now we just support fot one shot read mode
static ssize_t max30100_reg_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char data[4];
	int ir,red;
	int inter,frac;
	i2c_smbus_write_byte_data(max30100_client,MAX30100_FIFO_WR_PTR,++i);
	readSensor(data);
	ir = (data[0] << 8) | data[1];
	red = (data[2] << 8) | data[3];
        inter = i2c_smbus_read_byte_data(max30100_client,MAX30100_TEMP_INTG);
        frac = i2c_smbus_read_byte_data(max30100_client,MAX30100_TEMP_FRAC);
	return sprintf(buf,"MAX30100_REV_ID = %x,MAX30100_PART_ID = %x,IR = %x,RED = %d,temp = %d,%d\n",getRevID(),getPartID(),ir,red,inter,frac);

}

static ssize_t max30100_reg_set(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(max30100, 0644 , max30100_reg_show, NULL);
static struct attribute *max30100_attrs[] = {
    &dev_attr_max30100.attr,
    NULL
};
static struct attribute_group max30100_attr_group = {
    .name = "max30100_show",
    .attrs = max30100_attrs,
};


static int max30100_dev_init(void)
{
	char ret;
	printk("%s called\n", __func__);
	/*
	ret = i2c_smbus_read_byte_data(max30100_client,MAX30100_REV_ID);
	printk("MAX30100_REV_ID = %x\n",ret);
	ret = i2c_smbus_read_byte_data(max30100_client,MAX30100_PART_ID);
	printk("MAX30100_PART_ID = %x\n",ret);
	*/
	reset();
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_FIFO_WR_PTR,0);
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_FIFO_RD_PTR,0);
	ret = i2c_smbus_write_byte_data(max30100_client,MAX30100_OVRFLOW_CTR,0);
	mdelay(100);
	setLEDs(0x3,0xf,0xf);
	setSPO2(0x1);
	startup();
	/*
	i2c_smbus_write_byte_data(max30100_client, MAX30100_MODE_CONFIG, 0x02);
	i2c_smbus_write_byte_data(max30100_client, MAX30100_LED_CONFIG, 0xff);
	i2c_smbus_write_byte_data(max30100_client, MAX30100_SPO2_CONFIG, (0x1 <<6) | (0x1 << 2) | 0x3);
	*/
	return 0;
}

static int max30100_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
	max30100_client = i2c;
	max30100_dev_init();
	printk("max30100 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &max30100_attr_group);
	return 0;
}
static int max30100_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &max30100_attr_group);
	return 0;
}

static const struct i2c_device_id max30100_id[] = {  
    { "max30100", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, max30100_id);

static struct of_device_id max30100_of_match[] = {
        { .compatible = "maxim,max30100"},
        { }
};
MODULE_DEVICE_TABLE(of, max30100_of_match);

struct i2c_driver max30100_driver = {
    .driver = {
        .name           = "max30100",
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(max30100_of_match),
    },
    .probe      = max30100_probe,
    .remove     = max30100_remove,
    .id_table   = max30100_id,
};
module_i2c_driver(max30100_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-max30100 driver for testing module ");
MODULE_VERSION(DRV_VERSION);
