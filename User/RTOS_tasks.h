#include "FreeRTOS.h"
#include "task.h"

/* 线程参数 */
//优先级 0 ~ configMAX_PRIORITIES-1(6)
// #define TEST_TASK_PRIO       4      
// #define TEST_TASK_SIZE       256    
#define PRINT_TASK_PRIO      3      //线程优先级，数越大优先级越高
#define PRINT_STK_SIZE       512    //线程栈空间
#define SoftStart_PRIO       4
#define SoftStart_SIZE       256
#define ControlHandle_PRIO   0
#define ControlHandle_SIZE   2048
#define IMU_PRIO             1
#define IMU_SIZE             2048
#define DPS310_PRIO          2
#define DPS310_SIZE          256

extern void RTOS_init();
