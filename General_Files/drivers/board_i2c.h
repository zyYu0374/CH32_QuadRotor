#ifndef _BOARD_I2C_H_
#define _BOARD_I2C_H_
/*
新的软件模拟I2C，适配FreeRTOS
E:\#资料\FreeRTOS_SoftwareI2C\I2C\board_i2c.c
*/


/*********************************************************************
 * INCLUDES
 */
#include "ch32v30x.h"

/*********************************************************************
 * DEFINITIONS
 */
// I2C_SCL时钟
#define IIC_SCL_CLK         RCC_APB2Periph_GPIOD        // GPIO端口时钟
#define IIC_SCL_PORT        GPIOD                       // GPIO端口
#define IIC_SCL_PIN         GPIO_Pin_7                  // GPIO引脚
// I2C_SDA时钟
#define IIC_SDA_CLK         RCC_APB2Periph_GPIOD        // GPIO端口时钟
#define IIC_SDA_PORT        GPIOD                       // GPIO端口
#define IIC_SDA_PIN         GPIO_Pin_6                  // GPIO引脚

/*********************************************************************
 * MACROS
 */
#define IIC_SCL_0()         GPIO_ResetBits(IIC_SCL_PORT, IIC_SCL_PIN) 
#define IIC_SCL_1()         GPIO_SetBits(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SDA_0()         GPIO_ResetBits(IIC_SDA_PORT, IIC_SDA_PIN) 
#define IIC_SDA_1()         GPIO_SetBits(IIC_SDA_PORT, IIC_SDA_PIN) 
#define IIC_SDA_READ()      GPIO_ReadInputDataBit(IIC_SDA_PORT, IIC_SDA_PIN) 

/*********************************************************************
 * API FUNCTIONS
 */
void IIC_Init(void);			 
void IIC_Start(void);
void IIC_Start_Init(void);
void IIC_Stop(void);
void IIC_Stop_Init(void);//
void IIC_SendByte(uint8_t ucByte);
void IIC_SendByte_Init(uint8_t ucByte);
uint8_t IIC_ReadByte(void);
uint8_t IIC_ReadByte_Init(void);
uint8_t IIC_WaitAck(void);
uint8_t IIC_WaitAck_Init(void);
void IIC_Ack(void);
void IIC_Ack_Init(void);
void IIC_NAck(void);
uint8_t IIC_CheckDevice(uint8_t address);

#endif /* _BOARD_I2C_H_ */
