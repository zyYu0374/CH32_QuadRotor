#ifndef  CONTROL_HANDLE_H
#define CONTROL_HANDLE_H

#include "FreeRTOS.h"
#include "task.h"
#include "control_handle.h"
#include "pwm.h"
#include "pid.h"
#include "Crsf.h"
#include "IMU_handle.h"
#include "../General_Files/drivers/MTF01.h"

/*控制参数*/
#define CONRTOL_PERIOD  8  //控制周期，10ms->100Hz

extern void control_handle_task(void *pvParameters);


/*控制参数*/

#define single_Fly_Pitch_Zero  0.0f
#define single_Fly_Roll_Zero   0.0f
#define single_Fly_Yaw_Zero    0.0f


//共使用了Crsf的7个通道
#define ELRS_Throttle       CrsfChannels[2]//174~1805
#define ELRS_Pitch          CrsfChannels[1]//elevator
#define ELRS_Yaw            CrsfChannels[3]//rudder方向舵   //25.6.12:roll和yaw互换
#define ELRS_Roll           CrsfChannels[0]//aileron副翼

#define ELRS_Control_mode   CrsfChannels[6]//三个档位191,997,1792,右侧三档拨钮
#define ELRS_Throttle_lock  CrsfChannels[5]//三个档位191,997,1792,左侧三档拨钮
#define ELRS_NC             CrsfChannels[4]//松开191,按下1792
//#define ELRS_XXX            CrsfChannels[5]
//#define ELRX_XXX            CrsfChannels[7]

//控制模式 ELRS_Control_mode
enum
{
    PID_CONTROL_MODE = 1,   //电机转速为PID输出
    RAW_CONTROL_MODE,       //电机转速直接为油门输入（Debug模式）
    STABLE_CONTROL_MODE     //不可控，自稳定模式
};

//电机模式
enum
{
    MOTOR_NORMAL = 1,       //正常模式
    MOTOR_SOFT_STARTING     //正在软启动中
};

//全局变量
typedef struct
{
    float Mech_zero_yaw;    // yaw轴机械零点，因为需要更新所以是变量
    u8 is_locked;           // 电机锁
    u8 flight_mode;         //飞行模式
    u8 is_landing;          //自动降落

    PID_STRUCT PID_yaw_outerloop;
    PID_STRUCT PID_yaw_innerloop;
    PID_STRUCT PID_pitch_outerloop;
    PID_STRUCT PID_pitch_innerloop;
    PID_STRUCT PID_roll_outerloop;
    PID_STRUCT PID_roll_innerloop;
    PID_STRUCT MTF01_roll_outerloop;
    PID_STRUCT MTF01_roll_innerloop;
    PID_STRUCT MTF01_pitch_outerloop;
    PID_STRUCT MTF01_pitch_innerloop;
    PID_STRUCT MTF01_height_positionloop;

    uint16_t PWM_Out1;         // 最终作用到电机1的PWM
    uint16_t PWM_Out2;         // 最终作用到电机2的PWM
    uint16_t PWM_Out3;         // 最终作用到电机3的PWM
    uint16_t PWM_Out4;         // 最终作用到电机4的PWM

    float Yaw;
    float Pitch;
    float Roll;
    float MTF01_roll_agnle;
    float MTF01_pitch_agnle;
    uint16_t Throttle;  //1350~2232
    int CONTROL_MODE;    //控制模式设定
    int MOTOR_MODE;      //电机模式设定
}Control_TypeDef;


//电机软启动相关?
#define SOFT_START_TIME 500 //软启动时间，单位 ms   

#define IMU_SAMPLE_SIZE 4 //IMU平均值滤波器大小

// 积分限幅
#define Angle_I_Limit 200
#define Gyro_I_Limit  200

// ELRS数据转换到角度数据，ELRS_data * ELRS2angle = angle，30/(1811-1000) = 0.037
#define ELRS2angle    0.06
// ELRS数据转换到油门数据：ELRS_data * ELRS2throttle = throttle，1440/1711=0.8416
#define ELRS2throttle 0.8419  // 不是0.8419是因为略微留一点裕度给油门调节起飞
//#define ELRS2throttle 0.92  // 不是0.8419是因为略微留一点裕度给油门调节起飞

// 最大允许倾斜角度，还需计算
#define MAX_ROLL_ANGLE  20
#define MAX_PITCH_ANGLE 20

// 机械零点，需要调
#define Mech_zero_pitch  0
#define Mech_zero_roll   0

// 需要给油门初始调节留余量PWM，所以油门为100时不能等于7200空值
// 油门达到100时，PWM为 100*PWM_OIL
#define PWM_OIL 45

// 处于Locked状态时，油门和pid被禁用
#define Locked    1
#define Unlocked  0

// 飞行模式 ELRS_mode ，自由、稳定、GPS模式
#define Free    0
#define Stable  1
#define GPS     2

//调试壳子有阻尼，调试壳测出来的参数需要乘以一个比例系数
// #define damp_rate   0.55
#define damp_rate   1.00

// 着陆状态标志
#define landing      1
#define not_landing  0

// 定高（单位：cm）
#define stable_height 700

extern Control_TypeDef control;

extern void Stop_motor();


void control_handle_task(void *pvParameters);
void PIDSTRUCT_Init();
float ELRS_Convert_angle(int ELRS_data);
u16 ELRS_Convert_throttle(unsigned ELRS_data);
void ELRS_Convert_flight_mode();
void ELRS_Convert_lock();
void Update_ELRS();
void Roll_outerloop_ctr(float angle_num);
void Roll_innerloop_ctr();
void Yaw_outerloop_ctr(float angle_num);
void Yaw_innerloop_ctr();
void Pitch_outerloop_ctr(float angle_num);
void Pitch_innerloop_ctr();
void Flight_control();
void Stop_motor();
void Check_control_mode();
void control_para_init();
float angle2rad(float angle);



#endif
