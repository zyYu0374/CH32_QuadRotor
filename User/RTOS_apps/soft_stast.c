/****************************Print_status.c***************************************
负责电机的缓启动，防止电机启动过快挂掉

当检测到电机解锁时，会抢占电机的控制权，把电机的转速缓慢提升到怠速后再释放控制权


*******************************************************************************/

#include "soft_start.h"
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/pwm.h"
#include "control_handle.h"

int last_RC_lock_state = Locked;	//上一次锁定状态
float Motor_speed_set = PWM_THROTTLE_MIN; //油门值设定

void Motor_sort_start(void *pvParameters);

/*缓启动线程*/
void Motor_sort_start(void *pvParameters) 
{
    uint8_t LED1_MOTOR_cnt = 0;
    control.is_locked = Locked; //确保不会刚开机就缓启动（此线程优先级最高）
    while(1)
    {
        LED1_MOTOR_cnt ++;
        if(LED1_MOTOR_cnt == 10)
            GPIO_ResetBits(GPIOD,GPIO_Pin_8);//LED1
        if(control.is_locked == Unlocked && last_RC_lock_state == Locked){	//当解锁电机时候
            printf("MOTOR UNLOCKED!!!!\n");
            last_RC_lock_state = Unlocked;
            control.MOTOR_MODE = MOTOR_SOFT_STARTING;	//切换到缓启动模式
            // control.Mech_zero_yaw = MPU6050_para_filted.yaw;
            for(int i=0; i<SOFT_START_TIME; i++){	//缓启动逻辑
                //从最小油门PWM_THROTTLE_MIN一点一点加到电机最小转动油门PWM_THROTTLE_MIN_ROTATE
                Motor_speed_set = PWM_THROTTLE_MIN + ((((float)PWM_THROTTLE_MIN_ROTATE-(float)PWM_THROTTLE_MIN)/(float)SOFT_START_TIME))*i;
                    Motor_ctr_SOFT_START(Motor_speed_set, 1);
                    Motor_ctr_SOFT_START(Motor_speed_set, 2);
                    Motor_ctr_SOFT_START(Motor_speed_set, 3);
                    Motor_ctr_SOFT_START(Motor_speed_set, 4);

                    if(control.is_locked == Locked){    //处理缓启动中锁定电机
                        control.MOTOR_MODE = MOTOR_NORMAL;
                        break;
                    }

                vTaskDelay(1);
            }//301ms

            printf("MOTOR OK!!!!\n");
            control.Mech_zero_yaw = MPU6050_para_filted.yaw;    //解锁电机时设置此时方向为Yaw的零点
            control.MOTOR_MODE = MOTOR_NORMAL; //切换回正常模式
        }

        if(control.is_locked == Locked){
            last_RC_lock_state = Locked;
            control.MOTOR_MODE = MOTOR_NORMAL;
            Stop_motor();
//            control.Mech_zero_yaw = MPU6050_para_filted.yaw;
        }

        if(LED1_MOTOR_cnt == 20)
        {
            GPIO_SetBits(GPIOD,GPIO_Pin_8);LED1_MOTOR_cnt = 0;
            LED1_MOTOR_cnt == 0;
        }
            
        vTaskDelay(10);    //10ms监听一次
    }

}
