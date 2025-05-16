#include "debug.h"
#include "DPS310.h"
#include "stdint.h"
#include "MyI2C.h"

DPS310 DPS310_INSTANCE1;//实例化
DPS310* dps310 = &DPS310_INSTANCE1;	//全局结构体变量 用来保存从芯片内ROM读出的补偿参数

void DPS310_Init(void)
{
    MyI2C_Init();

    u8 Lsb,Msb;
	u8 MiddleByte;
	
	/********************1.读出矫正参数*********************/
	//头四个非16位
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c0_Least4Bits_REG);//低4位
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c0_MSB_REG);//高8位
	dps310->c0 = (((u16)Msb)<<4) + Lsb;			//0000 (xxxx xxxx) xxxx
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c1_LSB_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c1_Most4Bits_REG);
	dps310->c1 = (((u16)Msb)<<8) + Lsb;			//0000 (xxxx) xxxx xxxx
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c00_Least4Bits_REG);
	MiddleByte = DPS310_Read_Byte(DPS310_CALI_COEF_c00_MiddleByte_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c00_MSB_REG);
	dps310->c00 = (((u32)Msb)<<12) + ((u16)MiddleByte<<4) + Lsb;
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c10_LSB_REG);
	MiddleByte = DPS310_Read_Byte(DPS310_CALI_COEF_c10_MiddleByte_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c10_Most4Bits_REG);
	dps310->c10 = (((u32)Msb)<<16) + ((u16)MiddleByte<<8) + Lsb;	
	
	//标准16位
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c01_LSB_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c01_MSB_REG);
	dps310->c01 = (((u16)Msb)<<8) + Lsb;	
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c11_LSB_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c11_MSB_REG);
	dps310->c11 = (((u16)Msb)<<8) + Lsb;	
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c20_LSB_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c20_MSB_REG);
	dps310->c20 = (((u16)Msb)<<8) + Lsb;	
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c21_LSB_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c21_MSB_REG);
	dps310->c21 = (((u16)Msb)<<8) + Lsb;	
	Lsb = DPS310_Read_Byte(DPS310_CALI_COEF_c30_LSB_REG);
	Msb = DPS310_Read_Byte(DPS310_CALI_COEF_c30_MSB_REG);
	dps310->c30 = (((u16)Msb)<<8) + Lsb;	

	// printf("Calibration paramet:\r\n");
	// printf("c0:%d\r\nc1:%d\r\nc00:%d\r\nc10:%d\r\nc01:%d\r\nc11:%d\r\n",dps310->c0,dps310->c1,dps310->c00,dps310->c10,dps310->c01,dps310->c11);

	/***********************2.写入配置*************************/
	DPS310_Write_Byte(DPS310_RESET_REG,DPS310_RESET_FIFO_FLUSH|DPS310_RESET_SOFT_RST_VALUE);	//往复位寄存器写入给定值

	DPS310_Write_Byte(DPS310_PRS_CFG_REG,DPS310_PRS_CFG_PM_RATE_4_MEAS|DPS310_PRS_CFG_PM_PRC_8_TIMES);//气压采样率和过采样率
	DPS310_Write_Byte(DPS310_TMP_CFG_REG,DPS310_TMP_CFG_TMP_RATE_2_MEAS|DPS310_TMP_CFG_TMP_PRC_SINGLE);//温度采样率和过采样率
	DPS310_Write_Byte(DPS310_CFG_REG_REG,DPS310_CFG_RET_INT_MASK|DPS310_CFG_RET_FIFO_EN);

}


/************************* DPS310读写数据 ***************************/
uint8_t DPS310_Read_Byte(u8 reg)
{
	uint8_t rec_data;
	MyI2C_Start();
	MyI2C_SendByte(DPS310_ADDRESS<<1|0);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(reg);
	MyI2C_ReceiveAck();

	MyI2C_Start();
	MyI2C_SendByte(DPS310_ADDRESS<<1|1);
	MyI2C_ReceiveAck();
	rec_data = MyI2C_ReceiveByte();	//不应答
	MyI2C_Stop();
	return rec_data;
}

void DPS310_Write_Byte(u8 reg,u8 data)
{
	MyI2C_Start();
	MyI2C_SendByte(DPS310_ADDRESS<<1);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(reg);
	MyI2C_ReceiveAck();

	MyI2C_SendByte(data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}
uint8_t DPS310_ReadID(void)
{
	return DPS310_Read_Byte(DPS310_PRODUCT_ID_REG);
}

/******************************** 上层函数 *********************************/
//[7]COEF_RDY [6]SENSOR_RDY [5]TMP_RDY [4]PRS_RDY     [3]占位 [2][1][0]模式配置位
uint8_t DPS310_GetStatus(uint8_t isPRS_flag)
{
	uint8_t status;
	status = DPS310_Read_Byte(DPS310_MEAS_CFG_REG);//xx11 xxxx & 0010 0000
	switch(isPRS_flag)
	{
		case 0://temperature
			status = (status & 0x20) >> 5;
			break;
		case 1:
			status = (status & 0x10) >> 4;
			break;
		default:
			break;
	}
	return status;
}

//气压值-Pa
uint32_t DPS310_Get_Pressure(void)
{
	uint32_t Kp,Kt;
	Kp = DPS310_Get_Kp();
	Kt = DPS310_Get_Kt();

	uint8_t XLsb,Lsb,Msb;
	uint32_t Msb_32;
	uint32_t P_raw,T_raw;//原始值
	uint32_t P_raw_sc,T_raw_sc;//放缩后的值
	uint32_t P_cmpd;//补偿后的值
	DPS310_Write_Byte(DPS310_MEAS_CFG_REG,DPS310_MEAS_CFG_MEAS_CTRL_TMP);
	XLsb = DPS310_Read_Byte(DPS310_TMP_B0_REG);
	Lsb	 = DPS310_Read_Byte(DPS310_TMP_B1_REG);
	Msb	 = DPS310_Read_Byte(DPS310_TMP_B2_REG);
	T_raw = (((uint32_t)Msb) << 16) | (((uint16_t)Lsb) << 8) | XLsb;	//补偿前的温度值T_raw
	printf("Msb=%d\r\n", Msb);
	T_raw_sc = T_raw / Kt;
	DPS310_Write_Byte(DPS310_MEAS_CFG_REG,DPS310_MEAS_CFG_MEAS_CTRL_PRS);
	XLsb = DPS310_Read_Byte(DPS310_PSR_B0_REG);
	Lsb	 = DPS310_Read_Byte(DPS310_PSR_B1_REG);
	Msb	 = DPS310_Read_Byte(DPS310_PSR_B2_REG);
	P_raw = (((uint32_t)Msb) << 16) | (((uint16_t)Lsb) << 8) | XLsb;	//补偿前的气压值P_raw
	P_raw_sc = P_raw / Kp;

	// printf("T_raw=%d\r\n", T_raw);
	// Msb_32 = ((uint32_t)Msb) << 16;
	// printf("Msb_32=%d\r\n", Msb_32);
	// printf("P_raw=%d\r\n", P_raw);

	P_cmpd = DPS310_Compensate_P(P_raw_sc,T_raw_sc);
	return P_cmpd;
}

//温度值-℃
uint32_t DPS310_Get_Temperature(void)
{
	uint32_t Kt;
	Kt = DPS310_Get_Kt();

	uint8_t XLsb,Lsb,Msb;
	uint32_t T_raw;//原始值
	uint32_t T_raw_sc;//放缩后的值
	uint32_t T_cmpd;//补偿后的值
	XLsb = DPS310_Read_Byte(DPS310_TMP_B0_REG);
	Lsb	 = DPS310_Read_Byte(DPS310_TMP_B1_REG);
	Msb	 = DPS310_Read_Byte(DPS310_TMP_B2_REG);
	T_raw = (((uint32_t)Msb) << 16) | (((uint16_t)Lsb) << 8) | XLsb;	//补偿前的温度值T_raw
	T_raw_sc = T_raw / Kt;

	T_cmpd = DPS310_Compensate_T(T_raw_sc);
	return T_cmpd;
}

/****************************** 过程函数 *******************************/

uint32_t DPS310_Compensate_P(uint32_t P_raw_sc ,uint32_t T_raw_sc)
{
	uint32_t P_compensated;
	P_compensated = dps310->c00 + P_raw_sc*(dps310->c10 + P_raw_sc*(dps310->c20 + P_raw_sc*dps310->c30)) 
					+ T_raw_sc*dps310->c01 + T_raw_sc*P_raw_sc*(dps310->c11 + P_raw_sc*dps310->c21);
	return P_compensated;
}

uint32_t DPS310_Compensate_T(uint32_t T_raw_sc)
{
	uint32_t T_compensated;
	T_compensated = dps310->c0 * 0.5 + dps310->c1 * T_raw_sc;
	return T_compensated;
}

uint32_t DPS310_Get_Kp(void)
{
	uint8_t OSampleRate_P;
	OSampleRate_P = (DPS310_Read_Byte(DPS310_PRS_CFG_REG)) & 0x0F;//0x00~0x07
	switch(OSampleRate_P)
	{
		case 0x00:
			return 524288;
			break;
		case 0x01:
			return 1572864;
			break;
		case 0x02:
			return 3670016;
			break;
		case 0x03:
			return 7864320;
			break;
		case 0x04:
			return 253952;
			break;
		case 0x05:
			return 516096;
			break;
		case 0x06:
			return 1040384;
			break;
		case 0x07:
			return 2088960;
			break;
		default:
			break;
	}
}

uint32_t DPS310_Get_Kt(void)
{
	uint8_t OSampleRate_T;
	OSampleRate_T = (DPS310_Read_Byte(DPS310_TMP_CFG_REG)) & 0x0F;//0x00~0x07
	switch(OSampleRate_T)
	{
		case 0x00:
			return 524288;
			break;
		case 0x01:
			return 1572864;
			break;
		case 0x02:
			return 3670016;
			break;
		case 0x03:
			return 7864320;
			break;
		case 0x04:
			return 253952;
			break;
		case 0x05:
			return 516096;
			break;
		case 0x06:
			return 1040384;
			break;
		case 0x07:
			return 2088960;
			break;
		default:
			break;
	}
}
/***************************************END OF LINE*********************************************/