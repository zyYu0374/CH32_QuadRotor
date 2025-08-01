/****************************RTOS_tasks.c***************************************
用于创建和管理RTOS线程

添加新线程的方法：
1.在RTOS_apps中添加对应的.c和.h文件，并且写好线程主函数
2.在RTOS_tasks.c中声明一个TaskHandle_t结构体，并且在RTOS_tasks.h中规定好线程的优先级和栈空间
3.在RTOS_tasks.c中include线程对应的.h文件
4.在RTOS_init()中添加线程的初始化函数

*******************************************************************************/

#include "RTOS_tasks.h"
/*线程文件*/
#include "RTOS_apps/soft_start.h"
#include "RTOS_apps/Print_status.h"
#include "RTOS_apps/control_handle.h"
#include "RTOS_apps/IMU_handle.h"
#include "RTOS_apps/DPS310_handle.h"


TaskHandle_t PrintTask_Handler;
TaskHandle_t Motor_SoftStart_Handler;
TaskHandle_t Control_task_Handler;
TaskHandle_t IMU_Task_Handler;
TaskHandle_t DPS310_Task_Handler;


void RTOS_init();
void test_task(void *pvParameters);

void RTOS_init()
{
    // 测试线程
    // xTaskCreate((TaskFunction_t )test_task,
    //                 (const char*    )"test",
    //                 (uint16_t       )TEST_STK_SIZE,
    //                 (void*          )NULL,
    //                 (UBaseType_t    )TEST_TASK_PRIO,
    //                 (TaskHandle_t*  )&TESTTask_Handler);

    // 串口调试线程
    xTaskCreate((TaskFunction_t )Print_status_task,//100Hz
                (const char*    )"uart",
                (uint16_t       )PRINT_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )PRINT_TASK_PRIO,//3
                (TaskHandle_t*  )&PrintTask_Handler);
    
    // 电机软启动线程
    xTaskCreate((TaskFunction_t )Motor_sort_start,//100Hz
                (const char*    )"Soft",
                (uint16_t       )SoftStart_SIZE,
                (void*          )NULL,
                (UBaseType_t    )SoftStart_PRIO,//4
                (TaskHandle_t*  )&Motor_SoftStart_Handler);

    //飞机控制线程
    xTaskCreate((TaskFunction_t )control_handle_task,//125Hz
                (const char*    )"control",
                (uint16_t       )ControlHandle_SIZE,
                (void*          )NULL,
                (UBaseType_t    )ControlHandle_PRIO,//0
                (TaskHandle_t*  )&Control_task_Handler);

    //IMU线程
    xTaskCreate((TaskFunction_t )IMU_task,//200Hz；角速度获取200Hz，角度解算200Hz/2=100Hz
                (const char*    )"imu",
                (uint16_t       )IMU_SIZE,
                (void*          )NULL,
                (UBaseType_t    )IMU_PRIO,//1
                (TaskHandle_t*  )&IMU_Task_Handler);

    // //DPS310气压计线程
    // xTaskCreate((TaskFunction_t )DPS310_task,//10Hz
    //             (const char*    )"DPS310",
    //             (uint16_t       )DPS310_SIZE,
    //             (void*          )NULL,
    //             (UBaseType_t    )DPS310_PRIO,
    //             (TaskHandle_t*  )&DPS310_Task_Handler);
    printf("RTOS_Init OK!\r\n");
    vTaskStartScheduler();
}

/*测试函数*/
void test_task(void *pvParameters)
{
    while(1)
    {
        printf("TEST_Called\n");
        vTaskDelay(5);
    }
}

