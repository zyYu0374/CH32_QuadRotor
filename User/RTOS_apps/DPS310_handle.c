#include "stdint.h"
#include "DPS310.h"
#include "DPS310_handle.h"

void DPS310_task(void *pvParameters)
{
    while(1)
    {
        // printf("5\r\n");
        // DPS310_Get_Pressure();
        double Tmp = DPS310_Get_Temperature();
        printf("temp:%f\r\n",Tmp);
        vTaskDelay(10);
    }

}