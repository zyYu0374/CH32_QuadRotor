#include "debug.h"
#include "DPS310.h"
#include "stdint.h"
#include "MyI2C.h"
#include "RTOS_tasks.h"
#include "board_i2c.h"

DPS310 DPS310_INSTANCE1;//实例化
DPS310* dps310 = &DPS310_INSTANCE1;	//全局结构体变量 用来保存从芯片内ROM读出的补偿参数

uint32_t DPS310_Pressure = 1;
uint8_t reg_val,reg_val_after;//调试用
double last_T_raw_sc = 0;

void DPS310_Init(void)
{
    // MyI2C_Init();
	IIC_Init();

    u8 Lsb,Msb;
	u8 MiddleByte;
	int32_t temp;
	
	/********************1.读出矫正参数*********************/
	//头四个非16位
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c0_Least4Bits_REG);//低4位
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c0_MSB_REG);//高8位
	dps310->c0 = (((u16)Msb)<<4) + ((Lsb & 0xF0)>>4);			//0000 (xxxx xxxx) xxxx
	temp = dps310->c0;
	getTwosComplement(&temp, 12);
	dps310->c0 = temp;
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c1_LSB_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c1_Most4Bits_REG);
	dps310->c1 = (((u16)Msb & 0x0F)<<8) | Lsb;			//0000 (xxxx) xxxx xxxx
	temp = dps310->c1;
	getTwosComplement(&temp, 12);
	dps310->c0 = temp;
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c00_Least4Bits_REG);
	MiddleByte = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c00_MiddleByte_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c00_MSB_REG);
	dps310->c00 = (((u32)Msb)<<12) | ((u16)MiddleByte<<4) | ((Lsb & 0xF0)>>4);
	temp = dps310->c00;
	getTwosComplement(&temp, 20);
	dps310->c00 = temp;
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c10_LSB_REG);
	MiddleByte = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c10_MiddleByte_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c10_Most4Bits_REG);
	dps310->c10 = (((u32)(Msb & 0x0F))<<16) | ((u32)MiddleByte<<8) | Lsb;	
	temp = dps310->c10;
	getTwosComplement(&temp, 20);
	dps310->c10 = temp;
	
	//标准16位
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c01_LSB_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c01_MSB_REG);
	dps310->c01 = (((u16)Msb)<<8) + Lsb;	
	temp = dps310->c01;
	getTwosComplement(&temp, 16);
	dps310->c01 = temp;
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c11_LSB_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c11_MSB_REG);
	dps310->c11 = (((u16)Msb)<<8) + Lsb;	
	temp = dps310->c11;
	getTwosComplement(&temp, 16);
	dps310->c11 = temp;
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c20_LSB_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c20_MSB_REG);
	dps310->c20 = (((u16)Msb)<<8) + Lsb;
	temp = dps310->c20;
	getTwosComplement(&temp, 16);
	dps310->c20 = temp;
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c21_LSB_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c21_MSB_REG);
	dps310->c21 = (((u16)Msb)<<8) + Lsb;	
	temp = dps310->c21;
	getTwosComplement(&temp, 16);
	dps310->c21 = temp;
	Lsb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c30_LSB_REG);
	Msb = DPS310_Read_Byte_Init(DPS310_CALI_COEF_c30_MSB_REG);
	dps310->c30 = (((u16)Msb)<<8) + Lsb;	
	temp = dps310->c30;
	getTwosComplement(&temp, 16);
	dps310->c30 = temp;

	printf("Calibration paramet:\r\n");
	printf("c0:%d\r\nc1:%d\r\nc00:%d\r\nc10:%d\r\nc01:%d\r\nc11:%d\r\n",dps310->c0,dps310->c1,dps310->c00,dps310->c10,dps310->c01,dps310->c11);

	/***********************2.写入配置*************************/

	DPS310_Write_Byte_Init(DPS310_RESET_REG,DPS310_RESET_SOFT_RST_VALUE);	//往复位寄存器写入给定值
	Delay_Ms(10);//不delay 下一句会写入失败

	// reg_val = DPS310_Read_Byte_Init(DPS310_PRS_CFG_REG);
	// printf("reg_val=%d\r\n", reg_val);
	// reg_val = DPS310_Read_Byte_Init(DPS310_TMP_CFG_REG);
	// printf("reg_val=%d\r\n", reg_val);
	// reg_val = DPS310_Read_Byte_Init(DPS310_CFG_REG_REG);
	// printf("reg_val=%d\r\n", reg_val);

	DPS310_Write_Byte_Init(DPS310_PRS_CFG_REG,DPS310_PRS_CFG_PM_RATE_8_MEAS|DPS310_PRS_CFG_PM_PRC_16_TIMES);//气压采样率和过采样率
	DPS310_Write_Byte_Init(DPS310_TMP_CFG_REG,DPS310_TMP_CFG_TMP_RATE_2_MEAS|DPS310_TMP_CFG_TMP_PRC_SINGLE);//温度采样率和过采样率
	DPS310_Write_Byte_Init(DPS310_CFG_REG_REG,DPS310_CFG_RET_INT_MASK|DPS310_CFG_RET_FIFO_EN);//使能FIFO
	DPS310_Write_Byte_Init(DPS310_MEAS_CFG_REG,DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_PRS_TMP);//background mode自动模式

	// reg_val = DPS310_Read_Byte_Init(DPS310_PRS_CFG_REG);
	// printf("reg_val=%d\r\n", reg_val);
	// reg_val = DPS310_Read_Byte_Init(DPS310_TMP_CFG_REG);
	// printf("reg_val=%d\r\n", reg_val);
	// reg_val = DPS310_Read_Byte_Init(DPS310_CFG_REG_REG);
	// printf("reg_val=%d\r\n", reg_val);

	// reg_val = DPS310_Read_Byte_Init(DPS310_MEAS_CFG_REG);
	// printf("MEAS_CFG=%d\r\n", reg_val);


	printf("dps310 init OK!\r\n");

}


/************************* DPS310读写数据 ***************************/
// uint8_t DPS310_Read_Byte(u8 reg)
// {
// 	uint8_t rec_data;
// 	MyI2C_Start();
// 	MyI2C_SendByte(DPS310_ADDRESS<<1|0);//I2C协议中 最低位0表示主机写1表示主机读。高7位是设备地址
// 	MyI2C_ReceiveAck();
// 	MyI2C_SendByte(reg);
// 	MyI2C_ReceiveAck();

// 	MyI2C_Start();
// 	MyI2C_SendByte(DPS310_ADDRESS<<1|1);
// 	MyI2C_ReceiveAck();
// 	rec_data = MyI2C_ReceiveByte();	//不应答
// 	MyI2C_Stop();
// 	return rec_data;
// }

// void DPS310_Write_Byte(u8 reg,u8 data)
// {
// 	MyI2C_Start();
// 	MyI2C_SendByte(DPS310_ADDRESS<<1);
// 	MyI2C_ReceiveAck();
// 	MyI2C_SendByte(reg);
// 	MyI2C_ReceiveAck();

// 	MyI2C_SendByte(data);
// 	MyI2C_ReceiveAck();
// 	MyI2C_Stop();
// }
uint8_t DPS310_ReadID(void)
{
	return DPS310_Read_Byte(DPS310_PRODUCT_ID_REG);
}

/* board_i2c.c 版 */
uint8_t DPS310_Read_Byte(u8 reg)
{
	uint8_t rec_data;
	IIC_Start();
	IIC_SendByte(DPS310_ADDRESS<<1|0);//I2C协议中 最低位0表示主机写1表示主机读。高7位是设备地址
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();

	IIC_Start();
	IIC_SendByte(DPS310_ADDRESS<<1|1);
	IIC_WaitAck();
	rec_data = IIC_ReadByte();	//不应答
	IIC_Stop();
	return rec_data;
}
uint8_t DPS310_Read_Byte_Init(u8 reg)
{
	uint8_t rec_data;
	IIC_Start_Init();
	IIC_SendByte_Init(DPS310_ADDRESS<<1|0);//I2C协议中 最低位0表示主机写1表示主机读。高7位是设备地址
	IIC_WaitAck_Init();
	IIC_SendByte_Init(reg);
	IIC_WaitAck_Init();

	IIC_Start_Init();
	IIC_SendByte_Init(DPS310_ADDRESS<<1|1);
	IIC_WaitAck_Init();
	rec_data = IIC_ReadByte_Init();	//不应答
	IIC_Stop_Init();
	return rec_data;
}

void DPS310_Write_Byte(u8 reg,u8 data)
{
	IIC_Start();
	IIC_SendByte(DPS310_ADDRESS<<1);
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();

	IIC_SendByte(data);
	IIC_WaitAck();
	IIC_Stop();
}
void DPS310_Write_Byte_Init(u8 reg,u8 data)
{
	IIC_Start_Init();
	IIC_SendByte_Init(DPS310_ADDRESS<<1);
	IIC_WaitAck_Init();
	IIC_SendByte_Init(reg);
	IIC_WaitAck_Init();

	IIC_SendByte_Init(data);
	IIC_WaitAck_Init();
	IIC_Stop_Init();
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
double DPS310_Get_Pressure(void)
{
	uint32_t Kp,Kt;
	Kp = DPS310_Get_Kp();
	Kt = DPS310_Get_Kt();

	uint8_t XLsb,Lsb,Msb;
	uint32_t P_raw,T_raw;//原始值
	double P_raw_sc,T_raw_sc;//放缩后的值
	double P_cmpd;//补偿后的值
	// vTaskDelay(1);
	// DPS310_Write_Byte(DPS310_MEAS_CFG_REG,DPS310_MEAS_CFG_MEAS_CTRL_TMP);
	XLsb = DPS310_Read_Byte(DPS310_TMP_B0_REG);
	Lsb	 = DPS310_Read_Byte(DPS310_TMP_B1_REG);
	Msb	 = DPS310_Read_Byte(DPS310_TMP_B2_REG);
	T_raw = (((uint32_t)Msb) << 16) | (((uint32_t)Lsb) << 8) | (uint32_t)XLsb;	//补偿前的温度值T_raw
	T_raw_sc = T_raw / (Kt*1.0);
	// printf("T_raw=%d\r\n", T_raw);
	// printf("T_raw_sc=%f\r\n", T_raw_sc);
	// DPS310_Write_Byte(DPS310_MEAS_CFG_REG,DPS310_MEAS_CFG_MEAS_CTRL_PRS);
	XLsb = DPS310_Read_Byte(DPS310_PRS_B0_REG);
	Lsb	 = DPS310_Read_Byte(DPS310_PRS_B1_REG);
	Msb	 = DPS310_Read_Byte(DPS310_PRS_B2_REG);
	P_raw = (((uint32_t)Msb) << 16) | (((uint32_t)Lsb) << 8) | (uint32_t)XLsb;	//补偿前的气压值P_raw
	P_raw_sc = P_raw / (Kp*1.0);

	
	// Msb_32 = ((uint32_t)Msb) << 16;
	// printf("Msb_32=%d\r\n", Msb_32);
	// printf("P_raw=%d\r\n", P_raw);

	P_cmpd = DPS310_Compensate_P(P_raw_sc,T_raw_sc);
	// printf("P_cmpd=%f\r\n", P_cmpd);
	return P_cmpd;
}

//温度值-℃
double DPS310_Get_Temperature(void)
{
	uint32_t Kp,Kt;
	Kp = DPS310_Get_Kp();
	Kt = DPS310_Get_Kt();

	uint8_t XLsb,Lsb,Msb;
	int32_t P_raw,T_raw;//原始值
	double P_raw_sc,T_raw_sc;//放缩后的值
	double P_cmpd,T_cmpd;//补偿后的值

	// DPS310_Write_Byte(DPS310_CFG_REG_REG,DPS310_MEAS_CFG_MEAS_CTRL_TMP);//command mode:tmp
	// vTaskDelay(10);
	// reg_val = DPS310_Read_Byte(DPS310_CFG_REG_REG);
	// printf("MEAS_CFG=%d\r\n", reg_val);

	// if(((DPS310_Read_Byte(DPS310_MEAS_CFG_REG) & 0x20)>>5) == 1)//TMP_RDY
	// {
	// 	printf("TMP_RDY!\r\n");
		XLsb = DPS310_Read_Byte(DPS310_PRS_B0_REG);
		Lsb	 = DPS310_Read_Byte(DPS310_PRS_B1_REG);
		Msb	 = DPS310_Read_Byte(DPS310_PRS_B2_REG);
		// printf("xlsb=%d\r\n", XLsb);
		
		// vTaskDelay(20);
		
		if((DPS310_Read_Byte(DPS310_CFG_REG_REG) & 0x02) != 0)
		{	
			//启用了FIFO
			// printf("FIFO enabled!\r\n");

			if((XLsb & 0x01) == 0)//最低有效位是0
			{
				// printf("TMP TMP!\r\n");
				T_raw = (((uint32_t)Msb) << 16) | (((uint32_t)Lsb) << 8) | (uint32_t)XLsb;
				getTwosComplement(&T_raw,24);//从无符号转换成有符号 193822 20位

				T_raw_sc = T_raw / (Kt*1.0);
				last_T_raw_sc = T_raw_sc;
				T_cmpd = DPS310_Compensate_T(T_raw_sc);

				// printf("T_raw = %d\r\n", T_raw);
				// printf("T_raw_sc = %f\r\n", T_raw_sc);
				// printf("T_cmpd = %f\r\n", T_cmpd);
				return T_cmpd;
			}
			else if((XLsb & 0x01) == 1)//最低有效位是1
			{
				// printf("PRS PRS!\r\n");
				P_raw = (((uint32_t)Msb) << 16) | (((uint32_t)Lsb) << 8) | (uint32_t)XLsb;
				getTwosComplement(&P_raw,24);

				P_raw_sc = P_raw / (Kp*1.0);
				P_cmpd = DPS310_Compensate_P(P_raw_sc,last_T_raw_sc);

				// printf("P_raw = %d\r\n", P_raw);
				// printf("P_raw_sc = %f\r\n", P_raw_sc);
				// printf("P_cmpd = %f\r\n", P_cmpd);
				
				return P_cmpd;
			}
		}
		else
		{
			return 0;
		}
	// }
}

/****************************** 补码转换 *******************************/

void getTwosComplement(int32_t *raw, uint8_t length)
{
	if (*raw & ((uint32_t)1 << (length - 1)))
	{
		*raw -= (uint32_t)1 << length;
	}
}

/****************************** 过程函数 *******************************/

double DPS310_Compensate_P(double P_raw_sc ,double last_T_raw_sc)
{
	double P_compensated;
	P_compensated = dps310->c00 + P_raw_sc*(dps310->c10 + P_raw_sc*(dps310->c20 + P_raw_sc*dps310->c30)) 
					+ last_T_raw_sc*(dps310->c01 + P_raw_sc*(dps310->c11 + P_raw_sc*dps310->c21));
	return P_compensated;
}

double DPS310_Compensate_T(double T_raw_sc)
{
	double T_compensated;
	// printf("c0=%d\r\n", dps310->c0);
	// printf("c1=%d\r\n", dps310->c1);
	// printf("T_raw_sc=%f\r\n", T_raw_sc);
	T_compensated = dps310->c0 * 0.5 + dps310->c1 * T_raw_sc;
	// printf("T_compensated=%f\r\n", T_compensated);
	return T_compensated;
}

uint32_t DPS310_Get_Kp(void)
{
	uint8_t OSampleRate_P;
	OSampleRate_P = (DPS310_Read_Byte(DPS310_PRS_CFG_REG)) & 0x0F;//0x00~0x07
	// printf("OSampleRate_P=%d\r\n", OSampleRate_P);
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
	// printf("OSampleRate_T=%d\r\n", OSampleRate_T);
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