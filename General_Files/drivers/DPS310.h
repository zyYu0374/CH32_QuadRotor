#ifndef __DPS310_H
#define __DPS310_H
//默认I2C接口

#define DPS310_ADDRESS 0x76 //从设备地址,SDO接地

#define DPS310_PSR_B2_REG 0x00 //压力数据（补码）byte012
#define DPS310_PSR_B1_REG 0x01
#define DPS310_PSR_B0_REG 0x02

#define DPS310_TMP_B2_REG 0x03 //温度数据（补码）byte012
#define DPS310_TMP_B1_REG 0x04
#define DPS310_TMP_B0_REG 0x05

#define DPS310_PRS_CFG_REG 0x06 //气压测量配置地址
//写入数据最高位不使用
//采样率 measurements/s
#define DPS310_PRS_CFG_PM_RATE_1_MEAS 0x00 
#define DPS310_PRS_CFG_PM_RATE_2_MEAS 0x10
#define DPS310_PRS_CFG_PM_RATE_4_MEAS 0x20
#define DPS310_PRS_CFG_PM_RATE_8_MEAS 0x30
#define DPS310_PRS_CFG_PM_RATE_16_MEAS 0x40
#define DPS310_PRS_CFG_PM_RATE_32_MEAS 0x50
#define DPS310_PRS_CFG_PM_RATE_64_MEAS 0x60
#define DPS310_PRS_CFG_PM_RATE_128_MEAS 0x70
#define DPS310_PRS_CFG_PM_RATE_MASK 0x70
// 过采样率（影响单次采样时长、精度、电流消耗）
#define DPS310_PRS_CFG_PM_PRC_SINGLE 0x00  // low precision
#define DPS310_PRS_CFG_PM_PRC_2_TIMES 0x01  // low power
#define DPS310_PRS_CFG_PM_PRC_4_TIMES 0x02
#define DPS310_PRS_CFG_PM_PRC_8_TIMES 0x03
#define DPS310_PRS_CFG_PM_PRC_16_TIMES 0x04 // standard
#define DPS310_PRS_CFG_PM_PRC_32_TIMES 0x05
#define DPS310_PRS_CFG_PM_PRC_64_TIMES 0x06 // high precision
#define DPS310_PRS_CFG_PM_PRC_128_TIMES 0x07

#define DPS310_TMP_CFG_REG 0x07//温度测量配置地址
//写入数据最高位
#define DPS310_TMP_CFG_REG_TMP_EXT_INTERNAL 0x00  //内部传感器 in ASIC
#define DPS310_TMP_CFG_REG_TMP_EXT_EXTERNAL 0x80  //外部传感器 in pressure sensor MEMS element
//采样率 measurements/s
#define DPS310_TMP_CFG_TMP_RATE_1_MEAS 0x00 
#define DPS310_TMP_CFG_TMP_RATE_2_MEAS 0x10
#define DPS310_TMP_CFG_TMP_RATE_4_MEAS 0x20
#define DPS310_TMP_CFG_TMP_RATE_8_MEAS 0x30
#define DPS310_TMP_CFG_TMP_RATE_16_MEAS 0x40
#define DPS310_TMP_CFG_TMP_RATE_32_MEAS 0x50
#define DPS310_TMP_CFG_TMP_RATE_64_MEAS 0x60
#define DPS310_TMP_CFG_TMP_RATE_128_MEAS 0x70
#define DPS310_TMP_CFG_TMP_RATE_MASK 0x70 //0111 0000
//过采样率（precision）
#define DPS310_TMP_CFG_TMP_PRC_SINGLE 0x00  // default - measurment time 3.6 ms
#define DPS310_TMP_CFG_TMP_PRC_2_TIMES 0x01
#define DPS310_TMP_CFG_TMP_PRC_4_TIMES 0x02
#define DPS310_TMP_CFG_TMP_PRC_8_TIMES 0x03
#define DPS310_TMP_CFG_TMP_PRC_16_TIMES 0x04
#define DPS310_TMP_CFG_TMP_PRC_32_TIMES 0x05
#define DPS310_TMP_CFG_TMP_PRC_64_TIMES 0x06
#define DPS310_TMP_CFG_TMP_PRC_128_TIMES 0x07


#define DPS310_MEAS_CFG_REG 0x08     //Sensor Operating Mode and Status (MEAS_CFG)

#define DPS310_MEAS_CFG_COEF_RDY_AVAILABLE 0x80
#define DPS310_MEAS_CFG_SENSOR_RDY_COMPLETE 0x40
#define DPS310_MEAS_CFG_TMP_RDY 0x20
#define DPS310_MEAS_CFG_PRS_RDY 0x10

#define DPS310_MEAS_CFG_MEAS_CTRL_IDLE 0x00//不测量
#define DPS310_MEAS_CFG_MEAS_CTRL_PRS 0x01//pressure measurement
#define DPS310_MEAS_CFG_MEAS_CTRL_TMP 0x02//temperature measurement
#define DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_PRS 0x05
#define DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_TMP 0x06
#define DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_PRS_TMP 0x07

#define DPS310_CFG_REG_REG 0x09         // Configuration of interupts, measurement data shift, and FIFO enable

#define DPS310_CFG_RET_INT_HL 0x80//根据SDO连接方式配置产生的中断信号的极性（高低电平有效）
#define DPS310_CFG_RET_INT_SEL 0x70//产生三个测量中断
#define DPS310_CFG_RET_INT_MASK 0x00//不启用中断
#define DPS310_CFG_RET_TMP_SHIFT_EN 0x08
#define DPS310_CFG_RET_PRS_SHIFT_EN 0x04
#define DPS310_CFG_RET_FIFO_EN 0x02
#define DPS310_CFG_RET_TMP_SPI_MODE 0x01

#define DPS310_INT_STS_REG 0x0A         // Interrupt status register. The register is cleared on read.
#define DPS310_INT_STS_INT_FIFO_FULL_MASK 0x04  
#define DPS310_INT_STS_INT_TMP_MASK 0x02
#define DPS310_INT_STS_INT_PRS_MASK 0x01

#define DPS310_FIFO_STS_REG 0x0B        // FIFO status register
#define DPS310_FIFO_STS_FIFO_FULL_MASK 0x02
#define DPS310_FIFO_STS_FIFO_EMPTY_MASK 0x01

#define DPS310_RESET_REG 0x0C          //Softreset Register复位寄存器地址
#define DPS310_RESET_FIFO_FLUSH 0x80
#define DPS310_RESET_SOFT_RST_VALUE 0x09

#define DPS310_PRODUCT_ID_REG 0x0D     //Product and Revision ID (CHIPID)
#define DPS310_PRODUCT_ID_VALUE 0x10
#define DPS310_PRODUCT_ID_REV_ID_MASK 0xF0
#define DPS310_PRODUCT_ID_PROD_ID_MASK 0x0F

/************ 补偿部分 ************
 * 
 * 温度数据用来补偿大气压
 * 
 **********************************/
#define DPS310_CALI_COEF_start_REG 0x10
/*calibration parameters 每个地址对应一个字节8bits*/
#define DPS310_CALI_COEF_c0_Least4Bits_REG          0x11 
#define DPS310_CALI_COEF_c0_MSB_REG                 0x10 //Most significant byte高字节
#define DPS310_CALI_COEF_c1_LSB_REG                 0x12 //least significant byte低字节
#define DPS310_CALI_COEF_c1_Most4Bits_REG           0x11
#define DPS310_CALI_COEF_c00_Least4Bits_REG         0x15
#define DPS310_CALI_COEF_c00_MiddleByte_REG         0x14
#define DPS310_CALI_COEF_c00_MSB_REG                0x13
#define DPS310_CALI_COEF_c10_LSB_REG                0x17
#define DPS310_CALI_COEF_c10_MiddleByte_REG         0x16
#define DPS310_CALI_COEF_c10_Most4Bits_REG          0x15
#define DPS310_CALI_COEF_c01_LSB_REG                0x19
#define DPS310_CALI_COEF_c01_MSB_REG                0x18
#define DPS310_CALI_COEF_c11_LSB_REG                0x1B
#define DPS310_CALI_COEF_c11_MSB_REG                0x1A
#define DPS310_CALI_COEF_c20_LSB_REG                0x1D
#define DPS310_CALI_COEF_c20_MSB_REG                0x1C
#define DPS310_CALI_COEF_c21_LSB_REG                0x1F
#define DPS310_CALI_COEF_c21_MSB_REG                0x1E
#define DPS310_CALI_COEF_c30_LSB_REG                0x21
#define DPS310_CALI_COEF_c30_MSB_REG                0x20
//Done
//考虑一下这个基于温度传感器数据的补偿怎么使用

#define DPS310_TMP_COEF_SRCE_REG 0x28   //表示校准参数是基于哪个内部温度传感器的
#define DPS310_TMP_COEF_SRCE_MASK 0x80
#define DPS310_TMP_COEF_SRCE_EXTERNAL 0x80 //(of pressure sensor MEMS element)
#define DPS310_TMP_COEF_SRCE_INTERNAL 0x00 // (of ASIC)

void DPS310_Init(void);

uint8_t DPS310_Read_Byte(u8 reg);

void DPS310_Write_Byte(u8 reg,u8 data);

uint8_t DPS310_ReadID(void);

uint8_t DPS310_GetStatus(uint8_t status_flag);

uint32_t DPS310_Get_Pressure(void);

uint32_t DPS310_Get_Temperature(void);

uint32_t DPS310_Compensate_P(uint32_t P_raw_sc ,uint32_t T_raw_sc);

uint32_t DPS310_Compensate_T(uint32_t T_raw_sc);

uint32_t DPS310_Get_Kp(void);

uint32_t DPS310_Get_Kt(void);

typedef struct 
{
    /* 补偿系数 */
    uint16_t c0;//12b
    uint16_t c1;//12b
    uint32_t c00;//20b
    uint32_t c10;//20b

    uint16_t c01;//16b
    uint16_t c11;//16b
    uint16_t c20;//16b
    uint16_t c21;//16b
    uint16_t c30;//16b

}DPS310;


#endif