#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6050_CNT	1
#define MPU6050_NAME "mpu6050"

struct mpu6050_accel {
	signed short x;
	signed short y;
	signed short z;
};/* 加速度 */

struct mpu6050_gyro {
	signed short x;
	signed short y;
	signed short z;
};/* 陀螺仪 */

struct mpu6050_data {
	struct mpu6050_accel accel;
	struct mpu6050_gyro gyro;
	signed short temp;
};

struct mpu6050_dev {
	dev_t devid;				/* 设备号 	 */
	struct cdev cdev;			/* cdev 	*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备 	 */
	struct device_node	*nd; 	/* 设备节点 */
	int major;					/* 主设备号 */
	void *private_data;			/* 私有数据 		*/
	struct regmap *mpu6050_regmap;
	//struct mutex lock;
};

static struct i2c_client *mpu6050_i2c_client;

static int mpu6050_write_onereg(uint32_t reg_addr, uint32_t data)
{
	struct mpu6050_dev *mpu6050dev = i2c_get_clientdata(mpu6050_i2c_client);
    return regmap_write(mpu6050dev->mpu6050_regmap, reg_addr, data);
}

static int mpu6050_read_onereg(uint32_t reg_addr, uint32_t *data)
{
	struct mpu6050_dev *mpu6050dev = i2c_get_clientdata(mpu6050_i2c_client);
    return regmap_read(mpu6050dev->mpu6050_regmap, reg_addr, data);
}

static int mpu6050_read_accel(struct mpu6050_accel *acc)
{
    int i, ret;
	uint32_t data[6];

    for (i = 0; i < 6; i++) {
        ret = mpu6050_read_onereg(MPU6050_RA_ACCEL_XOUT_H + i, &data[i]);
		if (ret)
			return -EIO;
    }
    acc->x = (signed short)(data[0] << 8) + (signed short)data[1];
    acc->y = (signed short)(data[2] << 8) + (signed short)data[3];
    acc->z = (signed short)(data[4] << 8) + (signed short)data[5];
    return ret;
}

static int mpu6050_read_gyro(struct mpu6050_gyro *gyro)
{
    int i, ret;
	uint32_t data[6];

    for (i = 0; i < 6; i++) {
        ret = mpu6050_read_onereg(MPU6050_RA_GYRO_XOUT_H + i, &data[i]);
		if (ret)
			return -EIO;
    }
    gyro->x = (signed short)(data[0] << 8) + (signed short)data[1];
    gyro->y = (signed short)(data[2] << 8) + (signed short)data[3];
    gyro->z = (signed short)(data[4] << 8) + (signed short)data[5];
    return ret;
}

static int mpu6050_read_temp(short *temp)
{
    int i = 0;
    int ret = 0;
    uint32_t data[2] = {0};

    for (i = 0; i < 2; i++) {
        ret = mpu6050_read_onereg(MPU6050_RA_TEMP_OUT_H + i, &data[i]);
		if (ret)
			return -EIO;
    }
    *temp = (signed short)(data[0] << 8) + (signed short)data[1];
    return ret;
}

static int mpu6050_open(struct inode *inode, struct file *fp)
{
	fp->private_data = i2c_get_clientdata(mpu6050_i2c_client);
	return 0;
}

static int mpu6050_release(struct inode *inode, struct file *fp)
{
	
	return 0;
}

static ssize_t mpu6050_read(struct file *fp, char __user *buf, size_t size, loff_t *off)
{
	int ret;
	struct mpu6050_data data;
	//struct mpu6050_dev *mpu6050dev = (struct mpu6050_dev *)fp->private_data;
	
	//mutex_lock(&mpu6050dev->lock);
	ret = mpu6050_read_accel(&data.accel);
	if (ret)
		return ret;
	ret = mpu6050_read_gyro(&data.gyro);
	if (ret)
		return ret;
	ret = mpu6050_read_temp(&data.temp);
	if (ret)
		return ret;
	//mutex_unlock(&mpu6050dev->lock);
	
	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;
	
	return 0;
}

static struct file_operations mpu6050_dev_fops = {
	.owner = THIS_MODULE,
	.open = mpu6050_open,
	.release = mpu6050_release,
	.read = mpu6050_read,
};

static void mpu6050_init(struct i2c_client *client)
{
	uint32_t value = 0;
	
	mpu6050_write_onereg(MPU6050_RA_PWR_MGMT_1, 0x80);
	mdelay(50);
	mpu6050_write_onereg(MPU6050_RA_PWR_MGMT_1, 0x01);
	mdelay(50);

	mpu6050_read_onereg(MPU6050_RA_WHO_AM_I, &value);
	dev_info(&client->dev, "mpu6050 ID = %#X\r\n", value);

	mpu6050_write_onereg(MPU6050_RA_SMPLRT_DIV, 0x00); 	/* 输出速率是内部采样率					*/
	mpu6050_write_onereg(MPU6050_RA_GYRO_CONFIG, 0x18); 	/* 陀螺仪±2000dps量程 				*/
	mpu6050_write_onereg(MPU6050_RA_ACCEL_CONFIG, 0x18); 	/* 加速度计±16G量程 					*/
	mpu6050_write_onereg(MPU6050_RA_CONFIG, 0x04); 		/* 陀螺仪低通滤波BW=20Hz 				*/
	mpu6050_write_onereg(MPU6050_RA_FF_THR, 0x04); /* 加速度计低通滤波BW=21.2Hz 			*/
	mpu6050_write_onereg(MPU6050_RA_PWR_MGMT_2, 0x00); 	/* 打开加速度计和陀螺仪所有轴 				*/
	// mpu6050_write_onereg(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT); 	/* 关闭低功耗 						*/
	mpu6050_write_onereg(MPU6050_RA_FIFO_EN, 0x00);		/* 关闭FIFO						*/
}

static bool mpu6050_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case MPU6050_RA_ACCEL_XOUT_H ... MPU6050_RA_GYRO_ZOUT_L:
			return true;
		default:
			return false;
	}
}
static const struct regmap_config mpu6050_regmap_config = {
        .reg_bits = 8,
        .val_bits = 8,
        .volatile_reg = mpu6050_volatile_reg,
};

static int mpu6050_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret;
	struct mpu6050_dev *mpu6050dev;
	dev_info(&client->dev, "%s start.\n", __func__);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"i2c check functionality error (need I2C_FUNC_I2C)\n");
		return -ENXIO;
	}
	mpu6050dev = devm_kzalloc(&client->dev, sizeof(*mpu6050dev), GFP_KERNEL);
	if (!mpu6050dev) {
		dev_err(&client->dev, "mpu6050 char dev devm_kzalloc failed.\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&mpu6050dev->devid, 0, MPU6050_CNT, MPU6050_NAME);
	if (ret) {
		dev_err(&client->dev, "mpu6050 char dev region err:%d\n", ret);
		return ret;
	}
	mpu6050dev->major = MAJOR(mpu6050dev->devid);
	
	cdev_init(&mpu6050dev->cdev, &mpu6050_dev_fops);
	ret = cdev_add(&mpu6050dev->cdev, mpu6050dev->devid, MPU6050_CNT);
	if (ret) {
		dev_err(&client->dev, "mpu6050 char dev add err:%d\n", ret);
		goto err_cdev_add;
	}
	
	mpu6050dev->class = class_create(THIS_MODULE, MPU6050_NAME);
	if (IS_ERR(mpu6050dev->class)) {
		ret = PTR_ERR(mpu6050dev->class);
		dev_err(&client->dev, "mpu6050 create class err:%d\n", ret);
		goto err_class_create;
	}

	mpu6050dev->device = device_create(mpu6050dev->class, NULL, mpu6050dev->devid, NULL, MPU6050_NAME);
	if (IS_ERR(mpu6050dev->device)) {
		ret = PTR_ERR(mpu6050dev->device);
		dev_err(&client->dev, "mpu6050 create device err:%d\n", ret);
		goto err_device_create;
	}
	
	mpu6050dev->private_data = client;
	
	mpu6050dev->mpu6050_regmap = devm_regmap_init_i2c(client, &mpu6050_regmap_config);
	if (IS_ERR(mpu6050dev->mpu6050_regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		ret = PTR_ERR(mpu6050dev->mpu6050_regmap);
		goto err_regmap;
	}
	//mutex_init(&mpu6050dev->lock);
	i2c_set_clientdata(client, mpu6050dev);
	mpu6050_i2c_client = client;

	mpu6050_init(client);
	return 0;
	
err_regmap:
	device_destroy(mpu6050dev->class, mpu6050dev->devid);
err_device_create:
	class_destroy(mpu6050dev->class);
err_class_create:
	cdev_del(&mpu6050dev->cdev);
err_cdev_add:
	unregister_chrdev_region(mpu6050dev->devid, MPU6050_CNT);
	return ret;
}

static int mpu6050_remove(struct i2c_client *client)
{
	struct mpu6050_dev *mpu6050dev = i2c_get_clientdata(client);

	device_destroy(mpu6050dev->class, mpu6050dev->devid);
	class_destroy(mpu6050dev->class);
	cdev_del(&mpu6050dev->cdev);
	unregister_chrdev_region(mpu6050dev->devid, MPU6050_CNT);
	mpu6050_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id mpu6050_device_id[] = {
	{"aeetes,mpu6050", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mpu6050_device_id);

static const struct of_device_id mpu6050_of_match[] = {
	{.compatible = "aeetes,mpu6050"},
	{},
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static struct i2c_driver i2c_mpu6050_driver = {
	.probe = mpu6050_probe,
	.remove = mpu6050_remove,
	.id_table = mpu6050_device_id,
	.driver = {
		.name = MPU6050_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mpu6050_of_match,
	},
};

module_i2c_driver(i2c_mpu6050_driver);
MODULE_AUTHOR("Aeetes");
MODULE_LICENSE("GPL");