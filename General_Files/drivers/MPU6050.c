#include "MPU6050.h"  
#include "debug.h"
#include "../../User/RTOS_apps/IMU_handle.h"
#include "../General_Files/eMPL/inv_mpu.h"
/*RTOS*/
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

// #define PRINTF 1

static void I2C2_HARD_Init(u32 bound,u16 host_addr)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitTSturcture = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);     //使能APB1 I2C2外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);    //使能APB2 GPIO外设时钟

    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;         //设置为复用开漏输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = bound;     //设置I2C速率
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = host_addr;     //指定主设备地址
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &I2C_InitTSturcture);

    I2C_Cmd(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    Delay_Ms(50);
}

void MPU6050_I2C_Mem_Write(unsigned char DEV_ADDR, unsigned char REG_ADDR, unsigned char len, unsigned char *buf)
{
    //产生起始信号

    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET);
    I2C_GenerateSTART(I2C2, ENABLE);
    //发送地址&写
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C2, DEV_ADDR, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    //发送数据
    I2C_SendData(I2C2, REG_ADDR);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    while(len--)
    {
        I2C_SendData(I2C2, *buf++);
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }
    I2C_GenerateSTOP(I2C2, ENABLE);
}

static void MPU6050_WriteByte(unsigned char REG_ADDR,unsigned char _data)
{
    MPU6050_I2C_Mem_Write(MPU6050_ADDR, REG_ADDR, 1, &_data);
}

//陀螺仪分频后作为mpu6050的采样率。采样率=1K/(DIV+1)
static void MPU6050_SetRate(int rate)
{
    MPU6050_WriteByte(MPU6050_REG_SMPLRT_DIV, (1000/rate)-1);//低通滤波器(DLPF)使能时，陀螺仪输出频率为1kHz

    //设置数字低通滤波器
    if(rate/2>=188)MPU6050_WriteByte(MPU6050_REG_CONFIG, 0);
    else if(rate/2>=98)MPU6050_WriteByte(MPU6050_REG_CONFIG, 2);
    else if(rate/2>=42)MPU6050_WriteByte(MPU6050_REG_CONFIG, 3);
    else if(rate/2>=20)MPU6050_WriteByte(MPU6050_REG_CONFIG, 4);
    else if(rate/2>=10)MPU6050_WriteByte(MPU6050_REG_CONFIG, 5);
    else MPU6050_WriteByte(MPU6050_REG_CONFIG, 6);
}

unsigned char MPU6050_Init()
{
//=============================【初始化】=================================
//返回值：初始化mpu_dmp_init()的错误码，0表示初始化成功
//=======================================================================
    u8 res;

    I2C2_HARD_Init(250000, 0x00);//原为400K，由于程序会卡在EVT6，故降低频率
	MPU6050_WriteByte(MPU6050_REG_PWR_MGMT1, 0x80);//复位
	Delay_Ms(100);
	MPU6050_WriteByte(MPU6050_REG_PWR_MGMT1, 0x00);//解除休眠状态
	MPU6050_SetRate(125);//设置MPU6050采样率，原为400Hz
	MPU6050_WriteByte(MPU6050_REG_ACCEL_CONFIG, 0x01 << 3);//0x00 = 2g;0x01 = 4g;0x02 = 8g;0x03 = 16g    原为0x00 << 3
	MPU6050_WriteByte(MPU6050_REG_GYRO_CONFIG, 0x03 << 3);//0x00 = ±250dps;0x01 = ±500dps;0x02 = ±1000dps;0x03 = ±2000dps
	MPU6050_WriteByte(MPU6050_REG_INT_EN, 0X00);	//关闭所有中断
	MPU6050_WriteByte(MPU6050_REG_USER_CTRL, 0X00);	//I2C主模式关闭
	MPU6050_WriteByte(MPU6050_REG_FIFO_EN, 0X00);	//关闭FIFO
	MPU6050_WriteByte(MPU6050_REG_INTBP_CFG, 0X80);	//INT引脚低电平有效
	MPU6050_WriteByte(MPU6050_REG_PWR_MGMT1, 0X01);//设置CLKSEL,PLL X轴为参考

#ifdef DMP
	res = mpu_dmp_init();
	printf("mpu_dmp_init err code:%d\r\n", res);
    if(!res)    printf("mpu dmp init OK!\r\n");
#endif

	return res;
}

uint16_t count_mem_read = 0;
uint8_t MPU6050_I2C_Mem_Read(unsigned char DEV_ADDR, unsigned char REG_ADDR, unsigned char len, unsigned char *buf)
{
    uint16_t timeout = I2C_TIMEOUT; //防止死循环

    //产生起始信号
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET);
    I2C_GenerateSTART(I2C2, ENABLE);
    //发送地址&写
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C2, DEV_ADDR, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))   //卡在这一行！！！
    {
        count_mem_read++;
        printf_uart6("count:%d\r\n",count_mem_read);//输出到一个不使用的串口uart6……神秘
        
        if(--timeout == 0)//超时处理
        {
            printf("Bug Happen!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            // printf("Bug Happen!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            // printf("Bug Happen!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            // printf("Bug Happen!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            // printf("Bug Happen!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            return I2C_NoSuccess; //超时返回
        }
    }
    count_mem_read = 0; //重置计数器

    //发送数据
    I2C_SendData(I2C2, REG_ADDR);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    //产生起始信号
    I2C_GenerateSTART(I2C2, ENABLE);
    //发送地址&读
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C2, DEV_ADDR, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while(len--)
    {
        if(len == 0)I2C_AcknowledgeConfig(I2C2, DISABLE);//NACK
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
        *buf = I2C_ReceiveData(I2C2);
        buf++;
    }
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);//ACK

    return I2C_Success;
}


unsigned char dataH_last, dataL_last;
unsigned int MPU6050_Get_Data(u8 REG_ADDR)
{
//============================【读高低寄存器值】================================
//参数：
//REG_ADDR:寄存器高位地址
//返回值:获取到的16位数据
//===========================================================================
	unsigned char dataH, dataL;

    if(MPU6050_I2C_Mem_Read(MPU6050_ADDR, REG_ADDR, 1, &dataH) && MPU6050_I2C_Mem_Read(MPU6050_ADDR, REG_ADDR + 1, 1, &dataL))
    {
        dataH_last = dataH; //保存上次读取的高位数据
        dataL_last = dataL; //保存上次读取的低位数据
        return (u16)(dataH<<8|dataL);
    }
	else
    {
        // printf("I2C TimeOut!Skip!I2C TimeOut!Skip!I2C TimeOut!Skip!\r\n");
        return (u16)(dataH_last<<8|dataL_last);//读取失败就是用上次成功读取的值，跳过这一次
    }

}

float MPU6050_Get_Temp()
{
//============================【获取温度】================================
//返回值:获取到的温度
//======================================================================
    return (36.53 + (short)MPU6050_Get_Data(MPU6050_REG_TEMP_OUT_H) / 340.0);
}

#ifdef DMP
unsigned char MPU6050_MPU_DMP_GetData(void)
{
//============================【获取DMP值】================================
//返回值:mpu_dmp_get_data()的错误码，0表示获取成功
//========================================================================
	float a,b,c;
	u8 res;

    res = mpu_dmp_get_data(&a,&b,&c);
    MPU6050_para.yaw = c;
    MPU6050_para.pitch = a;
    MPU6050_para.roll = b;
    MPU6050_para.av_yaw = MPU6050_Get_Data(MPU6050_REG_GYRO_ZOUT_H);
    MPU6050_para.av_pitch = MPU6050_Get_Data(MPU6050_REG_GYRO_XOUT_H);
    MPU6050_para.av_roll = MPU6050_Get_Data(MPU6050_REG_GYRO_YOUT_H)-20;
//    printf("yaw:%.1f\r\npitch:%.1f\r\nroll:%.1f\r\n",c,a,b);
//    printf("mpu_dmp_get_data err code:%d\r\n", res);

	return res;
}
#endif
/************************************不使用DMP库************************************************************/

#define sampleFreq	125.0f			// sample frequency in Hz
/*采样周期的一半，用于求解四元数微分方程时计算角增量
请确定自己的姿态调用周期: 8ms,即上面的sampleFreq: 125Hz*/
#define halfT 0.004f

//这里的Kp,Ki是用于控制加速度计修正陀螺仪积分姿态的速度
#define Kp 2.0f  		//2.0f
#define Ki 0.0016f  	//0.002f

int16_t Gyro[3], Acc[3];			//原始数据
// float Pitch, Roll, Yaw;				//俯仰、横滚、偏航
float gx=0, gy=0, gz=0;				//由角速度计 算的角速率
float ax=0, ay=0, az=0;				//由加速度计 算的加速度

float acc_sp[3];						//积分速度
float acc_sp_RHRH[3];				//处理后速度

float bd_gx=0, bd_gy=0, bd_gz=0;
float bd_ax=0, bd_ay=0, bd_az=0;

//初始姿态四元数(地理坐标系)，q(q0,q1,q2,q3)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    //最优估计四元数
float q0_yaw = 1.0f, q1_yaw = 0.0f, q2_yaw = 0.0f, q3_yaw = 0.0f;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象
//定义姿态解算误差的积分
//当前加计测得的重力加速度在三轴(x,y,z)上的分量,与当前姿态计算得来的重力在三轴上的分量的误差的积分
float xErrorInt = 0.0f, yErrorInt = 0.0f, zErrorInt = 0.0f;

/*
 * 姿态融合
 * 单位: 加速度m/s^2   角速度rad/s
*/
void ImuUpdate(float gx, float gy, float gz, float ax, float ay, float az)//g表陀螺仪，a表加计
{
  float norm;
	
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q1q1 = q1 * q1;
  float q1q3 = q1 * q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;	
  float vx, vy, vz;
  float ex, ey, ez;
	
	float	q0_yawq0_yaw = q0_yaw * q0_yaw;
	float	q1_yawq1_yaw = q1_yaw * q1_yaw;
	float	q2_yawq2_yaw = q2_yaw * q2_yaw;
	float	q3_yawq3_yaw = q3_yaw * q3_yaw;
	float	q1_yawq2_yaw = q1_yaw * q2_yaw;
	float	q0_yawq3_yaw = q0_yaw * q3_yaw;
	
	//**************************Yaw轴计算******************************
	//Yaw轴四元素的微分方程，先单独解出yaw的姿态
  q0_yaw = q0_yaw + (-q1_yaw * gx - q2_yaw * gy - q3_yaw * gz) * halfT;	//halfT采样时间的一半
  q1_yaw = q1_yaw + (q0_yaw * gx + q2_yaw * gz - q3_yaw * gy) * halfT;
  q2_yaw = q2_yaw + (q0_yaw * gy - q1_yaw * gz + q3_yaw * gx) * halfT;
  q3_yaw = q3_yaw + (q0_yaw * gz + q1_yaw * gy - q2_yaw * gx) * halfT;
	
	//规范化Yaw轴四元数
  norm = sqrt(q0_yawq0_yaw + q1_yawq1_yaw + q2_yawq2_yaw + q3_yawq3_yaw);
  q0_yaw = q0_yaw / norm;
  q1_yaw = q1_yaw / norm;
  q2_yaw = q2_yaw / norm;
  q3_yaw = q3_yaw / norm;
	
	if(ax * ay * az	== 0)//如果加速度数据无效，或者自由坠落，不结算
	return ;
	
	//规范化加速度计值
  norm = sqrt(ax * ax + ay * ay + az * az); 
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
	
	//估计重力方向和流量/变迁，重力加速度在机体系的投影
  vx = 2 * (q1q3 - q0q2);											
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
  //向量外积再相减得到差分就是误差
  ex = (ay * vz - az * vy) ;      
  ey = (az * vx - ax * vz) ;
  ez = (ax * vy - ay * vx) ;
 
	//对误差进行PI计算
  xErrorInt = xErrorInt + ex * Ki;			
  yErrorInt = yErrorInt + ey * Ki;
  zErrorInt = zErrorInt + ez * Ki;
 
  //校正陀螺仪
  gx = gx + Kp * ex + xErrorInt;					
  gy = gy + Kp * ey + yErrorInt;
  gz = gz + Kp * ez + zErrorInt;			
			
	//四元素的微分方程
  q0 = q0 + (-q1 * gx - q2	*	gy - q3	*	gz)	*	halfT;
  q1 = q1 + (q0	*	gx + q2	*	gz - q3	*	gy)	*	halfT;
  q2 = q2 + (q0	*	gy - q1	*	gz + q3	*	gx)	*	halfT;
  q3 = q3 + (q0	*	gz + q1	*	gy - q2	*	gx)	*	halfT;
 
  //规范化Pitch、Roll轴四元数
  norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
	
	//求解欧拉角
	MPU6050_para_filted.pitch = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1) * 57.3f;
	MPU6050_para_filted.roll = asin(-2 * q1q3 + 2 * q0q2) * 57.3f;
	MPU6050_para_filted.yaw = -atan2(2 * q1_yawq2_yaw + 2 * q0_yawq3_yaw, -2 * q2_yawq2_yaw - 2 * q3_yawq3_yaw + 1)	* 57.3f;
    // printf("%f,%f,%f\r\n",MPU6050_para.yaw,MPU6050_para.pitch,MPU6050_para.roll);
}

void wdvhc_get_data(uint8_t on)
{
	MPU6050ReadAcc(Acc);
	MPU6050ReadGyro(Gyro);

	//陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

	//加速度计量程为:±2g        获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±4g        获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±8g        获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±16g       获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

    //输出值=加速度值(g)*4096
	ax = (float)Acc[0] * 0.0001220703125f;// 1/8192
	ay = (float)Acc[1] * 0.0001220703125f;
	az = (float)Acc[2] * 0.0001220703125f;
    // printf("%f,%f,%f\r\n",ax,ay,az);

    //加速度数据用来融合算出角度数据
    bd_ax = 0.4975f*ax - 0.0706f;// 线性校正，6.16校准后很准
	bd_ay = 0.4971f*ay + 0.0213f;
	bd_az = 0.4902f*az + 0.1419f;
    // printf("%f,%f,%f\r\n",bd_ax,bd_ay,bd_az);

    //输出值=每秒转动的度数(°/s)*16.4
	bd_gx = (float)Gyro[0] * 0.0609756f;// 1/16.4
	bd_gy = (float)Gyro[1] * 0.0609756f;
	bd_gz = (float)Gyro[2] * 0.0609756f;
    
    // printf("%f,%f,%f\r\n",bd_gx,bd_gy,bd_gz);
    // printf("%f\r\n",bd_gy);

    bd_gx += 3.1213f;//offset校正
    bd_gy += 1.4940f;
    bd_gz += 1.1098f;
    //角速度
    MPU6050_para.av_pitch = bd_gx;
    MPU6050_para.av_roll = bd_gy;
    MPU6050_para.av_yaw = -bd_gz;
    // printf("%f,%f,%f\r\n",bd_gx,bd_gy,bd_gz);
	
    // *0.0174533 = ÷57.3
	LPF_1_(1.8f,0.002f ,bd_gx*0.0174533f, gx);//一阶低通滤波
	LPF_1_(1.8f,0.002f ,bd_gy*0.0174533f, gy);
	
	acc_sp[0] += (az-0.995f)/3.0f;
	acc_sp[1] += ax * 5.2f;
	acc_sp[2] -= ay * 5.2f;	
	
	if(on == 1)
		ImuUpdate(bd_gx*0.0174533f, bd_gy*0.0174533f, bd_gz*0.0174533f, bd_ax, bd_ay, bd_az);
}

/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    // MPU6050_ReadData(MPU6050_REG_ACCEL_XOUT_H, buf, 6);//从0x3B开始 读6字节，X Y Z
    MPU6050_I2C_Mem_Read(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 6, buf);//第三个参数表字节数？从0x3B开始 读6字节，X Y Z
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    // MPU6050_ReadData(MPU6050_REG_GYRO_XOUT_H,buf,6);
    MPU6050_I2C_Mem_Read(MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}
