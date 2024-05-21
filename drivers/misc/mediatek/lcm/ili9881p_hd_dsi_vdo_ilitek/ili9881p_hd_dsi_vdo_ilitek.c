/*
 * Reversed by zoggn@Hakonti 2020y Fck CoronaVirus
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

//#define LCM_ID (0x98)

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>


#ifndef CONFIG_FPGA_EARLY_PORTING
#define I2C_I2C_LCD_BIAS_CHANNEL 0
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL	/* for I2C channel 0 */
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info tps65132_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif

/*static struct i2c_client *tps65132_i2c_client;*/
struct i2c_client *tps65132_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct tps65132_dev {
	struct i2c_client *client;

};

static const struct i2c_device_id tps65132_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver tps65132_iic_driver = {
	.id_table = tps65132_id,
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "tps65132",
#if !defined(CONFIG_MTK_LEGACY)
			.of_match_table = lcm_of_match,
#endif
		   },
};

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	LCM_LOGI("tps65132_iic_probe\n");
	LCM_LOGI("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	tps65132_i2c_client = client;
	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	LCM_LOGI("tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

/*static int tps65132_write_bytes(unsigned char addr, unsigned char value)*/
#if !defined(CONFIG_ARCH_MT6797)
int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = { 0 };

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		LCM_LOGI("tps65132 write data fail !!\n");
	return ret;
}
#endif

static int __init tps65132_iic_init(void)
{
	LCM_LOGI("tps65132_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
	i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
#endif
	LCM_LOGI("tps65132_iic_init2\n");
	i2c_add_driver(&tps65132_iic_driver);
	LCM_LOGI("tps65132_iic_init success\n");
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	LCM_LOGI("tps65132_iic_exit\n");
	i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Mike Liu");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
//static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT									(1440)


#ifndef CONFIG_FPGA_EARLY_PORTING
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x4F, 1, {0x01} },
	{REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table init_setting[] = {
	{ 0xFF, 0x03, {0x98, 0x81, 0x05}},
    { 0xB2, 0x01, {0x70}},
    { 0x03, 0x01, {0x00}},
    { 0x04, 0x01, {0x24}},
    { 0x30, 0x01, {0xF7}},
    { 0x29, 0x01, {0x00}},
    { 0x2A, 0x01, {0x12}},
    { 0x38, 0x01, {0xA8}},
    { 0x1A, 0x01, {0x50}},
    { 0x52, 0x01, {0x5F}},
    { 0x54, 0x01, {0x28}},
    { 0x55, 0x01, {0x25}},
    { 0x26, 0x01, {0x02}},
    { 0x3D, 0x01, {0xA1}},
    { 0x1B, 0x01, {0x01}},
    { 0xFF, 0x03, {0x98, 0x81, 0x02}},
    { 0x42, 0x01, {0x2F}},
    { 0x01, 0x01, {0x50}},
    { 0x15, 0x01, {0x10}},
    { 0x57, 0x01, {0x00}},
    { 0x58, 0x01, {0x1C}},
    { 0x59, 0x01, {0x2B}},
    { 0x5A, 0x01, {0x14}},
    { 0x5B, 0x01, {0x18}},
    { 0x5C, 0x01, {0x2A}},
    { 0x5D, 0x01, {0x1E}},
    { 0x5E, 0x01, {0x1F}},
    { 0x5F, 0x01, {0x94}},
    { 0x60, 0x01, {0x1E}},
    { 0x61, 0x01, {0x2A}},
    { 0x62, 0x01, {0x7D}},
    { 0x63, 0x01, {0x19}},
    { 0x64, 0x01, {0x17}},
    { 0x65, 0x01, {0x4A}},
    { 0x66, 0x01, {0x20}},
    { 0x67, 0x01, {0x26}},
    { 0x68, 0x01, {0x4C}},
    { 0x69, 0x01, {0x5A}},
    { 0x6A, 0x01, {0x25}},
    { 0x6B, 0x01, {0x00}},
    { 0x6C, 0x01, {0x1C}},
    { 0x6D, 0x01, {0x2B}},
    { 0x6E, 0x01, {0x14}},
    { 0x6F, 0x01, {0x18}},
    { 0x70, 0x01, {0x2A}},
    { 0x71, 0x01, {0x1E}},
    { 0x72, 0x01, {0x1F}},
    { 0x73, 0x01, {0x94}},
    { 0x74, 0x01, {0x1E}},
    { 0x75, 0x01, {0x2A}},
    { 0x76, 0x01, {0x7D}},
    { 0x77, 0x01, {0x19}},
    { 0x78, 0x01, {0x17}},
    { 0x79, 0x01, {0x4A}},
    { 0x7A, 0x01, {0x20}},
    { 0x7B, 0x01, {0x26}},
    { 0x7C, 0x01, {0x4C}},
    { 0x7D, 0x01, {0x5A}},
    { 0x7E, 0x01, {0x25}},
    { 0xFF, 0x03, {0x98, 0x81, 0x01}},
    { 0x01, 0x01, {0x00}},
    { 0x02, 0x01, {0x00}},
    { 0x03, 0x01, {0x56}},
    { 0x04, 0x01, {0x13}},
    { 0x05, 0x01, {0x13}},
    { 0x06, 0x01, {0x0A}},
    { 0x07, 0x01, {0x05}},
    { 0x08, 0x01, {0x05}},
    { 0x09, 0x01, {0x1D}},
    { 0x0A, 0x01, {0x01}},
    { 0x0B, 0x01, {0x00}},
    { 0x0C, 0x01, {0x3F}},
    { 0x0D, 0x01, {0x29}},
    { 0x0E, 0x01, {0x29}},
    { 0x0F, 0x01, {0x1D}},
    { 0x10, 0x01, {0x1D}},
    { 0x11, 0x01, {0x00}},
    { 0x12, 0x01, {0x00}},
    { 0x13, 0x01, {0x08}},
    { 0x14, 0x01, {0x08}},
    { 0x15, 0x01, {0x00}},
    { 0x16, 0x01, {0x00}},
    { 0x17, 0x01, {0x00}},
    { 0x18, 0x01, {0x00}},
    { 0x19, 0x01, {0x00}},
    { 0x1A, 0x01, {0x00}},
    { 0x1B, 0x01, {0x00}},
    { 0x1C, 0x01, {0x00}},
    { 0x1D, 0x01, {0x00}},
    { 0x1E, 0x01, {0x40}},
    { 0x1F, 0x01, {0x88}},
    { 0x20, 0x01, {0x08}},
    { 0x21, 0x01, {0x01}},
    { 0x22, 0x01, {0x00}},
    { 0x23, 0x01, {0x00}},
    { 0x24, 0x01, {0x00}},
    { 0x25, 0x01, {0x00}},
    { 0x26, 0x01, {0x00}},
    { 0x27, 0x01, {0x00}},
    { 0x28, 0x01, {0x33}},
    { 0x29, 0x01, {0x03}},
    { 0x2A, 0x01, {0x00}},
    { 0x2B, 0x01, {0x00}},
    { 0x2C, 0x01, {0x00}},
    { 0x2D, 0x01, {0x00}},
    { 0x2E, 0x01, {0x00}},
    { 0x2F, 0x01, {0x00}},
    { 0x30, 0x01, {0x00}},
    { 0x31, 0x01, {0x00}},
    { 0x32, 0x01, {0x00}},
    { 0x33, 0x01, {0x00}},
    { 0x34, 0x01, {0x00}},
    { 0x35, 0x01, {0x00}},
    { 0x36, 0x01, {0x00}},
    { 0x37, 0x01, {0x00}},
    { 0x38, 0x01, {0x00}},
    { 0x39, 0x01, {0x0F}},
    { 0x3A, 0x01, {0x2A}},
    { 0x3B, 0x01, {0xC0}},
    { 0x3C, 0x01, {0x00}},
    { 0x3D, 0x01, {0x00}},
    { 0x3E, 0x01, {0x00}},
    { 0x3F, 0x01, {0x00}},
    { 0x40, 0x01, {0x00}},
    { 0x41, 0x01, {0xE0}},
    { 0x42, 0x01, {0x40}},
    { 0x43, 0x01, {0x0F}},
    { 0x44, 0x01, {0x31}},
    { 0x45, 0x01, {0xA8}},
    { 0x46, 0x01, {0x00}},
    { 0x47, 0x01, {0x08}},
    { 0x48, 0x01, {0x00}},
    { 0x49, 0x01, {0x00}},
    { 0x4A, 0x01, {0x00}},
    { 0x4B, 0x01, {0x00}},
    { 0x4C, 0x01, {0xB2}},
    { 0x4D, 0x01, {0x22}},
    { 0x4E, 0x01, {0x01}},
    { 0x4F, 0x01, {0xF7}},
    { 0x50, 0x01, {0x29}},
    { 0x51, 0x01, {0x72}},
    { 0x52, 0x01, {0x25}},
    { 0x53, 0x01, {0xB2}},
    { 0x54, 0x01, {0x22}},
    { 0x55, 0x01, {0x22}},
    { 0x56, 0x01, {0x22}},
    { 0x57, 0x01, {0xA2}},
    { 0x58, 0x01, {0x22}},
    { 0x59, 0x01, {0x01}},
    { 0x5A, 0x01, {0xE6}},
    { 0x5B, 0x01, {0x28}},
    { 0x5C, 0x01, {0x62}},
    { 0x5D, 0x01, {0x24}},
    { 0x5E, 0x01, {0xA2}},
    { 0x5F, 0x01, {0x22}},
    { 0x60, 0x01, {0x22}},
    { 0x61, 0x01, {0x22}},
    { 0x62, 0x01, {0xEE}},
    { 0x63, 0x01, {0x02}},
    { 0x64, 0x01, {0x0B}},
    { 0x65, 0x01, {0x02}},
    { 0x66, 0x01, {0x02}},
    { 0x67, 0x01, {0x01}},
    { 0x68, 0x01, {0x00}},
    { 0x69, 0x01, {0x0F}},
    { 0x6A, 0x01, {0x07}},
    { 0x6B, 0x01, {0x55}},
    { 0x6C, 0x01, {0x02}},
    { 0x6D, 0x01, {0x02}},
    { 0x6E, 0x01, {0x5B}},
    { 0x6F, 0x01, {0x59}},
    { 0x70, 0x01, {0x02}},
    { 0x71, 0x01, {0x02}},
    { 0x72, 0x01, {0x57}},
    { 0x73, 0x01, {0x02}},
    { 0x74, 0x01, {0x02}},
    { 0x75, 0x01, {0x02}},
    { 0x76, 0x01, {0x02}},
    { 0x77, 0x01, {0x02}},
    { 0x78, 0x01, {0x02}},
    { 0x79, 0x01, {0x02}},
    { 0x7A, 0x01, {0x0A}},
    { 0x7B, 0x01, {0x02}},
    { 0x7C, 0x01, {0x02}},
    { 0x7D, 0x01, {0x01}},
    { 0x7E, 0x01, {0x00}},
    { 0x7F, 0x01, {0x0E}},
    { 0x80, 0x01, {0x06}},
    { 0x81, 0x01, {0x54}},
    { 0x82, 0x01, {0x02}},
    { 0x83, 0x01, {0x02}},
    { 0x84, 0x01, {0x5A}},
    { 0x85, 0x01, {0x58}},
    { 0x86, 0x01, {0x02}},
    { 0x87, 0x01, {0x02}},
    { 0x88, 0x01, {0x56}},
    { 0x89, 0x01, {0x02}},
    { 0x8A, 0x01, {0x02}},
    { 0x8B, 0x01, {0x02}},
    { 0x8C, 0x01, {0x02}},
    { 0x8D, 0x01, {0x02}},
    { 0x8E, 0x01, {0x02}},
    { 0x8F, 0x01, {0x44}},
    { 0x90, 0x01, {0x44}},
    { 0xFF, 0x03, {0x98, 0x81, 0x06}},
    { 0x01, 0x01, {0x03}},
    { 0x04, 0x01, {0x70}},
    { 0x2B, 0x01, {0x0A}},
    { 0xC0, 0x01, {0xCF}},
    { 0xC1, 0x01, {0x2A}},
    { 0xFF, 0x03, {0x98, 0x81, 0x00}},
    { 0x35, 0x01, {0x00}},
    { 0x11, 0x01, {0x00}},
    { REGFLAG_DELAY, 120, {}},
    { 0x29, 0x01, {0x00}},
    { REGFLAG_DELAY, 20, {}}
};

#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A, 4, {0x00, 0x00, (FRAME_WIDTH >> 8), (FRAME_WIDTH & 0xFF)} },
	{0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT & 0xFF)} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Display off sequence */
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },

	/* Sleep Mode On */
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

  params->type = 2;
  params->width = FRAME_WIDTH;
  params->physical_width = 0;
  params->physical_height = 0;
  params->physical_width_um = 0;
  params->physical_height_um = 0;
  params->dsi.switch_mode = 0;
  params->dsi.switch_mode_enable = 0;
  params->dsi.LANE_NUM = 4;
  params->dsi.data_format.color_order = 0;
  params->dsi.data_format.trans_seq = 0;
  params->dsi.data_format.padding = 0;
  params->dsi.data_format.format = 2;
  params->dsi.PS = 2;
  params->dsi.horizontal_active_pixel = FRAME_WIDTH;
  params->dsi.vertical_sync_active = 8;
  params->dsi.ssc_disable = 0;
  params->dsi.vertical_backporch = 16;
  params->height = FRAME_HEIGHT;
  params->dsi.vertical_frontporch = 24;
  params->dsi.mode = 1;
  params->dsi.vertical_frontporch_for_low_power = 470;
  params->dsi.packet_size = 256;
  params->dsi.horizontal_sync_active = 20;
  params->dsi.horizontal_backporch = 20;
  params->dsi.horizontal_frontporch = 20;
  params->dsi.vertical_active_line = FRAME_HEIGHT;
  params->dsi.PLL_CLOCK = 204;
  params->dsi.PLL_CK_VDO = 204;
  params->dsi.PLL_CK_CMD = 220;
  params->dsi.CLK_HS_POST = 36;
  params->dsi.clk_lp_per_line_enable = 0;
  params->dsi.customization_esd_check_enable = 0;
  params->dsi.esd_check_enable = 1;
  params->dsi.lcm_esd_check_table[0].cmd = 10;
  params->dsi.lcm_esd_check_table[0].count = 1;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

#ifdef BUILD_LK
#ifndef CONFIG_FPGA_EARLY_PORTING
#define TPS65132_SLAVE_ADDR_WRITE  0x7C
static struct mt_i2c_t TPS65132_i2c;

static int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;

	TPS65132_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;	/* I2C2; */
	/* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
	TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
	TPS65132_i2c.mode = ST_MODE;
	TPS65132_i2c.speed = 100;
	len = 2;

	ret_code = i2c_write(&TPS65132_i2c, write_data, len);
	/* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

	return ret_code;
}

#else

/* extern int mt8193_i2c_write(u16 addr, u32 data); */
/* extern int mt8193_i2c_read(u16 addr, u32 *data); */

/* #define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data) */
/* #define TPS65132_read_byte(add)  mt8193_i2c_read(add) */

#endif
#endif


static void lcm_init_power(void)
{

}

static void lcm_suspend_power(void)
{

}

static void lcm_resume_power(void)
{

}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
#ifndef CONFIG_FPGA_EARLY_PORTING
	int ret = 0;
#endif

	cmd = 0x00;
	data = 0x0E;

	SET_RESET_PIN(0);

#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);
#else
	set_gpio_lcd_enp(1);
#endif
	MDELAY(5);
#ifdef BUILD_LK
	ret = TPS65132_write_byte(cmd, data);
#else
#if !defined(CONFIG_ARCH_MT6797)
	ret = tps65132_write_bytes(cmd, data);
#endif
#endif

	if (ret < 0)
		LCM_LOGI("ili9881c----tps6132----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("ili9881c----tps6132----cmd=%0x--i2c write success----\n", cmd);

	cmd = 0x01;
	data = 0x0E;

#ifdef BUILD_LK
	ret = TPS65132_write_byte(cmd, data);
#else
#if !defined(CONFIG_ARCH_MT6797)
	ret = tps65132_write_bytes(cmd, data);
#endif
#endif

	if (ret < 0)
		LCM_LOGI("ili9881c----tps6132----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("ili9881c----tps6132----cmd=%0x--i2c write success----\n", cmd);

#endif
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
	if (lcm_dsi_mode == CMD_MODE) {
		LCM_LOGI("ili9881c----not support ----lcm mode\n");
	} else {
		push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
		LCM_LOGI("ili9881c----tps6132----lcm mode = vdo mode :%d----\n", lcm_dsi_mode);
	}
}

static void lcm_suspend(void)
{
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ZERO);
#else
	set_gpio_lcd_enp(0);
#endif
#endif
	SET_RESET_PIN(0);
}

static void lcm_resume(void)
{
	lcm_init();
}


static unsigned int lcm_compare_id(void)
{
	return 1; // Dirty hack!
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

	LCM_LOGI("%s,ili9881c backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = level;

	push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

static void *lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
	if (mode == 0) {	/* V2C */
		lcm_switch_mode_cmd.mode = CMD_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;	/* mode control addr */
		lcm_switch_mode_cmd.val[0] = 0x13;	/* enabel GRAM firstly, ensure writing one frame to GRAM */
		lcm_switch_mode_cmd.val[1] = 0x10;	/* disable video mode secondly */
	} else {		/* C2V */
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		lcm_switch_mode_cmd.val[0] = 0x03;	/* disable GRAM and enable video mode */
	}
	return (void *)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}


LCM_DRIVER ili9881p_hd_dsi_vdo_ilitek_lcm_drv = {
	.name = "ili9881p_hd_dsi_vdo_ilitek_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.switch_mode = lcm_switch_mode,
};
