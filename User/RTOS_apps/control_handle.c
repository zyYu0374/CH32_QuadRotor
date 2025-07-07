/****************************control_handle.c***************************************
负责无人机的控制

原：
飞机电机对应图
       3号    4号
 坤头↑   \  /
        /  \
       2号    1号
pwm: 电机pwm
n:   电机编号 motor_ctr(pwm,n)

25嵌赛：
飞机电机序号(n)对应图
       1号    2号
 坤头↑   \  /
        /  \
       3号    4号
    <<(箭头丝印)
*******************************************************************************/

#include "control_handle.h"
#include "math.h"
#include <pwm.h>

/*全局变量*/
Control_TypeDef control;
extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload_filtered;
extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;


float Px_zero_point;
float Py_zero_point;
float compensate_factor = 1.0f;

void control_handle_task(void *pvParameters)
{
    uint8_t LED3_CTRL_cnt = 0;
    control_para_init();    // 初始化全局变量

    while(1)
    {
        LED3_CTRL_cnt ++;
        if(LED3_CTRL_cnt == 10)
            GPIO_ResetBits(GPIOA,GPIO_Pin_15);//LED3
        Update_ELRS();//遥控器数据更新
        if(control.MOTOR_MODE != MOTOR_SOFT_STARTING){      //如果电机正在缓启动，电机不执行控制
            if(control.is_locked==Unlocked){
                if(control.Throttle>=PWM_CLOSE_LOOP_CONTROL_ENABLE){
                    Flight_control();
                }
                else if(control.Throttle<PWM_CLOSE_LOOP_CONTROL_ENABLE){
                    Motor_ctr(control.Throttle,1);//以ELRS_Convert_throttle里转化的油门值怠速
                    Motor_ctr(control.Throttle,2);
                    Motor_ctr(control.Throttle,3);
                    Motor_ctr(control.Throttle,4);
                }
            }
            else{
                Stop_motor();
                pid_func.clc(&control.PID_roll_outerloop);//clear
                pid_func.clc(&control.PID_roll_innerloop);
                pid_func.clc(&control.PID_pitch_outerloop);
                pid_func.clc(&control.PID_pitch_innerloop);
                pid_func.clc(&control.PID_yaw_outerloop);
                pid_func.clc(&control.PID_yaw_innerloop);
            }
        }

        if(LED3_CTRL_cnt == 20)//关LED
        {
            GPIO_SetBits(GPIOA,GPIO_Pin_15);
            LED3_CTRL_cnt = 0;
        }
            
        vTaskDelay(CONRTOL_PERIOD);
    }
}

void PIDSTRUCT_Init()
{
    /*
     * 陀螺仪的角速度是角速度越大，寄存器值越小，所以内环的PID参数是负数
     * 航向角由于右手定则，通常取反，所以PID参数是负
     */
    /******************************* Yaw *************************************/
    // 航向角外环初始化（角度环）
    pid_func.reset(&control.PID_yaw_outerloop);
    control.PID_yaw_outerloop.Kp=2.0f*damp_rate;//3.0
    control.PID_yaw_outerloop.Ki=0.0f*damp_rate;
    control.PID_yaw_outerloop.Kd=0.0f*damp_rate;
    control.PID_yaw_outerloop.max_iout=Angle_I_Limit;
    control.PID_yaw_outerloop.min_iout=-Angle_I_Limit;
    control.PID_yaw_outerloop.max_out=65535;
    control.PID_yaw_outerloop.min_out=-65535;
    control.PID_yaw_outerloop.DeadBand = 0.01;    //PID死区
    pid_func.init(&control.PID_yaw_outerloop);      // 清空缓存

    // 航向角内环初始化（角速度环）
    pid_func.reset(&control.PID_yaw_innerloop);
    control.PID_yaw_innerloop.Kp=1.7f*damp_rate;//2.7
    control.PID_yaw_innerloop.Ki=0.0f*damp_rate;
    control.PID_yaw_innerloop.Kd=4.0f*damp_rate;//6.9
    control.PID_yaw_innerloop.max_iout=Gyro_I_Limit;
    control.PID_yaw_innerloop.min_iout=-Gyro_I_Limit;
    control.PID_yaw_innerloop.max_out=65535;
    control.PID_yaw_innerloop.min_out=-65535;
    control.PID_yaw_innerloop.DeadBand=0.01;
    pid_func.init(&control.PID_yaw_innerloop);      // 清空缓存

    /******************************* pitch *************************************/
    // 俯仰角外环初始化（角度环）
    pid_func.reset(&control.PID_pitch_outerloop);
    control.PID_pitch_outerloop.Kp=2.0f*damp_rate;
    control.PID_pitch_outerloop.Ki=0.0f*damp_rate; //-0.12
    control.PID_pitch_outerloop.Kd=0.0f*damp_rate;  //-5.3
    control.PID_pitch_outerloop.max_iout=Angle_I_Limit;
    control.PID_pitch_outerloop.min_iout=-Angle_I_Limit;
    control.PID_pitch_outerloop.max_out=65535;
    control.PID_pitch_outerloop.min_out=-65535;
    control.PID_pitch_outerloop.DeadBand = 0.01;    //PID死区
    pid_func.init(&control.PID_pitch_outerloop);    // 清空缓存

    // 俯仰角内环初始化（角速度环）
    pid_func.reset(&control.PID_pitch_innerloop);
    control.PID_pitch_innerloop.Kp=0.52f*damp_rate;    //2.2
    // control.PID_pitch_innerloop.Ki=0.029f*damp_rate;    //0.0
    control.PID_pitch_innerloop.Kd=1.9*damp_rate;    //5.7
    control.PID_pitch_innerloop.max_iout=Gyro_I_Limit;
    control.PID_pitch_innerloop.min_iout=-Gyro_I_Limit;
    control.PID_pitch_innerloop.max_out=300;
    control.PID_pitch_innerloop.min_out=-300;
    control.PID_pitch_innerloop.DeadBand=0.01;
    pid_func.init(&control.PID_pitch_innerloop);    // 清空缓存

    /******************************* Roll *************************************/
    // 横滚角外环初始化（角度环）
    pid_func.reset(&control.PID_roll_outerloop);
    control.PID_roll_outerloop.Kp=2.0*damp_rate;//2.3
    control.PID_roll_outerloop.Ki=0.0*damp_rate;
    control.PID_roll_outerloop.Kd=0.0*damp_rate;//0.38
    control.PID_roll_outerloop.max_iout=Angle_I_Limit;
    control.PID_roll_outerloop.min_iout=-Angle_I_Limit;
    control.PID_roll_outerloop.max_out=65535;
    control.PID_roll_outerloop.min_out=-65535;
    control.PID_roll_outerloop.DeadBand=0.005;
    pid_func.init(&control.PID_roll_outerloop);     // 清空缓存

    // 横滚角内环初始化（角速度环）
    //第一个调
    pid_func.reset(&control.PID_roll_innerloop);
    control.PID_roll_innerloop.Kp=0.52f*damp_rate;//0.825  0.785  0.77  0.52
    control.PID_roll_innerloop.Ki=0.029f;//0.0198  0.018
    control.PID_roll_innerloop.Kd=1.9*damp_rate;//2.3，之前有的时候被输出限幅了
    control.PID_roll_innerloop.max_iout=Gyro_I_Limit;
    control.PID_roll_innerloop.min_iout=-Gyro_I_Limit;
    control.PID_roll_innerloop.max_out=300;
    control.PID_roll_innerloop.min_out=-300;
    control.PID_roll_innerloop.DeadBand=0.01;
    pid_func.init(&control.PID_roll_innerloop);     // 清空缓存

    //////////////////////////////////////////MTF01 roll///////////////////////////////////////////////////////////////////

    // 采用角度控制，也即把光流计引入的控制量作为飞机pitch和roll来补偿
    // MTF01横滚角外环初始化（位置环）
    pid_func.reset(&control.MTF01_roll_outerloop);
    control.MTF01_roll_outerloop.Kp=0.1f;
    control.MTF01_roll_outerloop.Ki=0.0f;
    control.MTF01_roll_outerloop.Kd=0.0f;
    control.MTF01_roll_outerloop.max_iout=Angle_I_Limit;
    control.MTF01_roll_outerloop.min_iout=-Angle_I_Limit;
    control.MTF01_roll_outerloop.max_out=65535;
    control.MTF01_roll_outerloop.min_out=-65535;
    control.MTF01_roll_outerloop.DeadBand=0.01;
    pid_func.init(&control.MTF01_roll_outerloop);

    // MTF01横滚角内环初始化（速度环）
    pid_func.reset(&control.MTF01_roll_innerloop);
    control.MTF01_roll_innerloop.Kp=0.1;
    control.MTF01_roll_innerloop.Ki=0.0f;
    control.MTF01_roll_innerloop.Kd=0.0;
    control.MTF01_roll_innerloop.max_iout=Gyro_I_Limit;
    control.MTF01_roll_innerloop.min_iout=-Gyro_I_Limit;
    control.MTF01_roll_innerloop.max_out=10;
    control.MTF01_roll_innerloop.min_out=-10;
    control.MTF01_roll_innerloop.DeadBand=1;
    pid_func.init(&control.MTF01_roll_innerloop);

    //////////////////////////////////////////MTF01 pitch///////////////////////////////////////////////////////////////////
    // MTF01俯仰角外环初始化（位置环）
    pid_func.reset(&control.MTF01_pitch_outerloop);
    control.MTF01_pitch_outerloop.Kp=0.1f;
    control.MTF01_pitch_outerloop.Ki=0.0f;
    control.MTF01_pitch_outerloop.Kd=3.9f;
    control.MTF01_pitch_outerloop.max_iout=Angle_I_Limit;
    control.MTF01_pitch_outerloop.min_iout=-Angle_I_Limit;
    control.MTF01_pitch_outerloop.max_out=65535;
    control.MTF01_pitch_outerloop.min_out=-65535;
    control.MTF01_pitch_outerloop.DeadBand=0.01;
    pid_func.init(&control.MTF01_pitch_outerloop);

    // MTF01俯仰角内环初始化（速度环）
    pid_func.reset(&control.MTF01_pitch_innerloop);
    control.MTF01_pitch_innerloop.Kp=0.1f;
    control.MTF01_pitch_innerloop.Ki=0.0f;
    control.MTF01_pitch_innerloop.Kd=0.0f;
    control.MTF01_pitch_innerloop.max_iout=Gyro_I_Limit;
    control.MTF01_pitch_innerloop.min_iout=-Gyro_I_Limit;
    control.MTF01_pitch_innerloop.max_out=10;
    control.MTF01_pitch_innerloop.min_out=-10;
    control.MTF01_pitch_innerloop.DeadBand=1;
    pid_func.init(&control.MTF01_pitch_innerloop);

    //////////////////////////////////////////MTF01 pitch///////////////////////////////////////////////////////////////////
    pid_func.reset(&control.MTF01_height_positionloop);
    control.MTF01_height_positionloop.Kp=3.1;
    control.MTF01_height_positionloop.Ki=0.0f;
    control.MTF01_height_positionloop.Kd=0.0;
    control.MTF01_height_positionloop.max_iout=Gyro_I_Limit;
    control.MTF01_height_positionloop.min_iout=-Gyro_I_Limit;
    control.MTF01_height_positionloop.max_out=65535;
    control.MTF01_height_positionloop.min_out=-65535;
    control.MTF01_height_positionloop.DeadBand=1;
    pid_func.init(&control.MTF01_height_positionloop);
}
//***********************************************************************
// 将摇杆值转化为角度，映射为±30°
float ELRS_Convert_angle(int ELRS_data)
{
    float angle;
    if(ELRS_data>1000){
        angle=(ELRS_data-1000)*ELRS2angle;//0 ~ +30°
    }
    else if(ELRS_data<985){
        angle=(ELRS_data-985)*ELRS2angle;//-30°
    }
    else {
        angle=0;
    }
    return angle;
}

// 将摇杆值转化为油门
u16 ELRS_Convert_throttle(unsigned ELRS_data)   //174~1805
{
    u16 throttle;
    if(ELRS_data<=200){
        throttle = PWM_THROTTLE_START_POINT;//缓启动后的怠速：1350
    }
    else {
        throttle = PWM_THROTTLE_START_POINT + (u16)(ELRS_data-200)*ELRS2throttle;
    }
    return throttle;
}

// 将摇杆值转化为电机锁
void ELRS_Convert_lock()
{
    if (ELRS_Throttle_lock>=1785 && ELRS_Throttle_lock<=1800){
        control.is_locked = Unlocked;
    }
    else if(ELRS_Throttle_lock>=990 && ELRS_Throttle_lock<=1010){
        control.is_locked = Locked;
    }
    else{
        control.is_locked = Locked;
    }
}

// // 将摇杆值转化为飞行模式
// void ELRS_Convert_flight_mode()
// {
//     if (ELRS_mode>=1785 && ELRS_mode<=1800){
//         control.flight_mode = GPS;
//     }
//     else if(ELRS_mode>=990 && ELRS_mode<=1010){
//         control.flight_mode = Stable;
//     }
//     else{
//         control.flight_mode = Free;
//     }
// }

// 更新运动控制模式
void Check_control_mode()
{
    if (ELRS_Control_mode>=1785 && ELRS_Control_mode<=1800){
        control.CONTROL_MODE = PID_CONTROL_MODE;
    }
    else if(ELRS_Control_mode>=990 && ELRS_Control_mode<=1010){
        control.CONTROL_MODE = RAW_CONTROL_MODE;
    }
    else{
        control.CONTROL_MODE = STABLE_CONTROL_MODE;
    }
}


// 更新遥控器发来的各个ELRS值
void Update_ELRS()
{
    control.Yaw=ELRS_Convert_angle(ELRS_Yaw);
    control.Roll=ELRS_Convert_angle(ELRS_Roll);
    control.Pitch=-ELRS_Convert_angle(ELRS_Pitch);
    control.Throttle = ELRS_Convert_throttle(ELRS_Throttle);
    ELRS_Convert_lock();
    // ELRS_Convert_flight_mode();
    Check_control_mode(); //同步控制模式
}

//***********************************************************************
// 由于倾斜会导致推力的竖直分量的损失，故需要对于油门进行补偿
float Throttle_compensate(float pitch, float roll)
{
    double z_vector[3];
    z_vector[0]=sin(angle2rad(pitch))*cos(angle2rad(roll));//把单位Z轴向量 [0, 0, 1]，经过 roll 和 pitch 的旋转后，转换到世界坐标系下。
    z_vector[1]=-sin(angle2rad(roll));
    z_vector[2]=cos(angle2rad(pitch))*cos(angle2rad(roll));

    float temp=z_vector[2]/sqrt(z_vector[0]*z_vector[0]+z_vector[1]*z_vector[1]+z_vector[2]*z_vector[2]);
    return temp;
}

//角度转弧度
float angle2rad(float angle)
{
    return angle*3.1416/180.0f;
}

//********************************************MPU6050******************************************************
// Roll控制
void Roll_outerloop_ctr(float angle_num)
{
    pid_func.calc(&control.PID_roll_outerloop, angle_num, MPU6050_para_filted.roll);
}

void Roll_innerloop_ctr()
{
    //av_roll是一个比较大的值，手转飞机时有三四百；除以100放缩
    pid_func.calc(&control.PID_roll_innerloop, control.PID_roll_outerloop.out, MPU6050_para_filted.av_roll);//Debug：除�?50.0f�?消除量纲，控制周�?20ms，�?�环单位�?度每�?
//    printf("d: %f, %f", PID_roll_outerloop.out, MPU6050_para_filted.av_roll/1000.0f);
}

// Yaw控制
void Yaw_outerloop_ctr(float angle_num)
{
    pid_func.calc(&control.PID_yaw_outerloop, angle_num, MPU6050_para_filted.yaw);
}

void Yaw_innerloop_ctr()
{
    pid_func.calc(&control.PID_yaw_innerloop, control.PID_yaw_outerloop.out, MPU6050_para_filted.av_yaw);
}

// Pitch控制
void Pitch_outerloop_ctr(float angle_num)
{
    pid_func.calc(&control.PID_pitch_outerloop, angle_num, MPU6050_para_filted.pitch);
}

void Pitch_innerloop_ctr()
{
    pid_func.calc(&control.PID_pitch_innerloop, control.PID_pitch_outerloop.out, MPU6050_para_filted.av_pitch);
}

//********************************************光流定点********************************************************

// X轴光流控制（双环）
void Px_outerloop_ctr()
{
    pid_func.calc(&control.MTF01_roll_outerloop, Px_zero_point, payload_filtered.Px);
}

void Px_innerloop_ctr()
{
    pid_func.calc(&control.MTF01_roll_innerloop, control.MTF01_roll_outerloop.out, payload_filtered.Vx);
}

// Y轴光流控制（双环）
void Py_outerloop_ctr()
{
    pid_func.calc(&control.MTF01_pitch_outerloop, Py_zero_point, payload_filtered.Py);
}

void Py_innerloop_ctr()
{
    pid_func.calc(&control.MTF01_pitch_innerloop, control.MTF01_pitch_outerloop.out, payload_filtered.Vy);
}

// 高度光流控制（单位置环）
void Pz_outerloop_ctr()
{
    pid_func.calc(&control.MTF01_pitch_outerloop, stable_height, payload_filtered.distance);
}

//*********************************************************************************************************

void Flight_control()
{
    control.Mech_zero_yaw = MPU6050_para_filted.yaw;     // 防止转向后机头回0

    //油门补偿 取绝对值+限幅
    // compensate_factor=Throttle_compensate(MPU6050_para_filted.pitch,  MPU6050_para_filted.roll);
    // if(compensate_factor<0)
    // {
    //     compensate_factor=-compensate_factor;
    // }
    // if(compensate_factor<0.707f)//arccos(0.85)=31.78deg cos(45°)=0.707  
    // {
    //     compensate_factor=0.707f;
    // }


    if(control.CONTROL_MODE == RAW_CONTROL_MODE)  // Debug模式
    {

        Motor_ctr(control.Throttle,1);
        Motor_ctr(control.Throttle,2);
        Motor_ctr(control.Throttle,3);
        Motor_ctr(control.Throttle,4);
    }
    else if(control.CONTROL_MODE == STABLE_CONTROL_MODE && payload.tof_status == 1)  // 当雷达数据可用时，才能进入自稳模式
    {
        /*在这里补充控制程序*/
        Px_outerloop_ctr();
        Px_innerloop_ctr();

        Py_outerloop_ctr();
        Py_innerloop_ctr();

        control.MTF01_roll_agnle = control.MTF01_roll_innerloop.out;
        control.MTF01_pitch_agnle = control.MTF01_pitch_innerloop.out;

        /*end*/

        Roll_outerloop_ctr(control.Roll + Mech_zero_roll + control.MTF01_roll_agnle);
        Roll_innerloop_ctr();

        Pitch_outerloop_ctr(control.Pitch + Mech_zero_pitch + control.MTF01_pitch_agnle);
        Pitch_innerloop_ctr();

        Yaw_outerloop_ctr((control.Yaw + control.Mech_zero_yaw));
        Yaw_innerloop_ctr();

        //Mixer
        control.PWM_Out1=control.Throttle/compensate_factor+control.PID_pitch_innerloop.out+control.PID_roll_innerloop.out-control.PID_yaw_innerloop.out;
        control.PWM_Out2=control.Throttle/compensate_factor+control.PID_pitch_innerloop.out-control.PID_roll_innerloop.out+control.PID_yaw_innerloop.out;
        control.PWM_Out3=control.Throttle/compensate_factor-control.PID_pitch_innerloop.out-control.PID_roll_innerloop.out-control.PID_yaw_innerloop.out;
        control.PWM_Out4=control.Throttle/compensate_factor-control.PID_pitch_innerloop.out+control.PID_roll_innerloop.out+control.PID_yaw_innerloop.out;

        Limit(control.PWM_Out1, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);
        Limit(control.PWM_Out2, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);
        Limit(control.PWM_Out3, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);
        Limit(control.PWM_Out4, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);

        Motor_ctr(control.PWM_Out1,1);
        Motor_ctr(control.PWM_Out2,2);
        Motor_ctr(control.PWM_Out3,3);
        Motor_ctr(control.PWM_Out4,4);
    }
    else if(control.CONTROL_MODE == PID_CONTROL_MODE || (control.CONTROL_MODE == STABLE_CONTROL_MODE && payload.tof_status == 0))  // 正常PID模式
    {
        Roll_outerloop_ctr(control.Roll + Mech_zero_roll);       // 是负的是因为调整了机头方向
        Roll_innerloop_ctr();
        // printf("%f,%f\r\n",control.PID_roll_innerloop.out,control.PID_roll_outerloop.out);

        Pitch_outerloop_ctr(control.Pitch + Mech_zero_pitch);    
        Pitch_innerloop_ctr();

        Yaw_outerloop_ctr(control.Yaw + control.Mech_zero_yaw);
        Yaw_innerloop_ctr();

        //PID模式 Mixer 6.17根据b站那个开源调整符号
        // control.PWM_Out1=control.Throttle/compensate_factor - control.PID_pitch_innerloop.out - control.PID_roll_innerloop.out + control.PID_yaw_innerloop.out;//--+
        // control.PWM_Out2=control.Throttle/compensate_factor - control.PID_pitch_innerloop.out + control.PID_roll_innerloop.out - control.PID_yaw_innerloop.out;//-+-
        // control.PWM_Out3=control.Throttle/compensate_factor + control.PID_pitch_innerloop.out - control.PID_roll_innerloop.out - control.PID_yaw_innerloop.out;//+--
        // control.PWM_Out4=control.Throttle/compensate_factor + control.PID_pitch_innerloop.out + control.PID_roll_innerloop.out + control.PID_yaw_innerloop.out;//+++

        //6.18调整，与上刚好相反
        //飞机向右转，roll_inner.out是个负值，所以1 3电机应该是油门+roll_inner.out,2 4电机是油门-roll_inner.out
        control.PWM_Out1=control.Throttle/compensate_factor + control.PID_pitch_innerloop.out + control.PID_roll_innerloop.out - control.PID_yaw_innerloop.out;//++-
        control.PWM_Out2=control.Throttle/compensate_factor + control.PID_pitch_innerloop.out - control.PID_roll_innerloop.out + control.PID_yaw_innerloop.out;//+-+
        control.PWM_Out3=control.Throttle/compensate_factor - control.PID_pitch_innerloop.out + control.PID_roll_innerloop.out + control.PID_yaw_innerloop.out;//-++
        control.PWM_Out4=control.Throttle/compensate_factor - control.PID_pitch_innerloop.out - control.PID_roll_innerloop.out - control.PID_yaw_innerloop.out;//---

        Limit(control.PWM_Out1, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);
        Limit(control.PWM_Out2, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);
        Limit(control.PWM_Out3, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);
        Limit(control.PWM_Out4, PWM_THROTTLE_MAX, PWM_THROTTLE_MIN);

        Motor_ctr(control.PWM_Out1,1);
        Motor_ctr(control.PWM_Out2,2);
        Motor_ctr(control.PWM_Out3,3);
        Motor_ctr(control.PWM_Out4,4);

        Px_zero_point=payload_filtered.Px;
        Py_zero_point=payload_filtered.Py;
    }
    else
    {
        Stop_motor();
    }
}

//初始化全局变量
void control_para_init()
{
    control.Mech_zero_yaw = 0;  // yaw轴机械零点，因为需要更新所以是变量
    control.is_locked = 1;      // 电机锁
    control.flight_mode = 1;    //飞行模式
    control.is_landing = 0;     //自动降落

    control.PWM_Out1=0;     // 最终作用到电机1的PWM
    control.PWM_Out2=0;     // 最终作用到电机2的PWM
    control.PWM_Out3=0;     // 最终作用到电机3的PWM
    control.PWM_Out4=0;     // 最终作用到电机4的PWM

    control.Yaw   = 0;
    control.Pitch = 0;
    control.Roll  = 0;
    control.Throttle = 0;
    control.CONTROL_MODE = PID_CONTROL_MODE;    //控制模式设定
    control.MOTOR_MODE = MOTOR_NORMAL;          //电机模式设定
}

void Stop_motor()
{
    Motor_ctr(PWM_THROTTLE_MIN,1);
    Motor_ctr(PWM_THROTTLE_MIN,2);
    Motor_ctr(PWM_THROTTLE_MIN,3);
    Motor_ctr(PWM_THROTTLE_MIN,4);
}


