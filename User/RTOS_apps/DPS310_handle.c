#include "stdint.h"
#include "DPS310.h"
#include "DPS310_handle.h"
#include "control_handle.h"
#include "MPU6050.h"  

void DPS310_task(void *pvParameters)
{
    // uint8_t ThrottleLock_LastSta = 1;
    // uint8_t ThrottleLock_status = 1;
    while(1)
    {
        // printf("5\r\n");
        // DPS310_Get_Pressure();
        // double Tmp = DPS310_Get_Temperature();
        // printf("temp:%f\r\n",Tmp);
        
        // ThrottleLock_status = control.is_locked;
        // if(ThrottleLock_status == 0 && ThrottleLock_LastSta == 1){
        //     MPU6050_WriteByte(MPU6050_REG_PWR_MGMT1, 0x80);//复位
        //     vTaskDelay(100);   
        // }

        // ThrottleLock_LastSta = ThrottleLock_status;
        vTaskDelay(100);
    }

}