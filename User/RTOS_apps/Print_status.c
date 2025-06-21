/****************************Print_status.c***************************************
从调试串口输出无人机的信�?


*******************************************************************************/
#include "stdint.h"
#include "DPS310.h"
#include "Print_status.h"

void Print_status_task(void *pvParameters);
void Graph_print();
void String_print();
void Serial_data_send();
void Serial_send_char(USART_TypeDef *USART, uint8_t *data, int size);
extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload_filtered;
extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
extern float compensate_factor;
SendPackType SendPack;  //向上位机发送的数据包

void Print_status_task(void *pvParameters)
{
    uint8_t LED2_PRINT_cnt = 0;
    while(1)
    {
        LED2_PRINT_cnt++;
        if(LED2_PRINT_cnt == 10)
            GPIO_ResetBits(GPIOB,GPIO_Pin_12);//LED2
        // printf("2\r\n");
        // printf("Print\r\n");

        // Graph_print();
        String_print();
        // Serial_data_send();

        if(LED2_PRINT_cnt == 20)
        {
            GPIO_SetBits(GPIOB,GPIO_Pin_12);
            LED2_PRINT_cnt == 0;
        }

        vTaskDelay(PRINT_DELAY_TIME);

    }
}

void Serial_data_send()
{
    SendPack.head = 0xEF;
    SendPack.end = 0x5A;

    /*IMU信息*/
    SendPack.IMU_yaw = MPU6050_para_filted.yaw;
    SendPack.IMU_pitch = MPU6050_para_filted.pitch;
    SendPack.IMU_roll = MPU6050_para_filted.roll;
    SendPack.IMU_av_yaw = MPU6050_para_filted.av_yaw;
    SendPack.IMU_av_pitch = MPU6050_para_filted.av_pitch;
    SendPack.IMU_av_roll = MPU6050_para_filted.av_roll;

    /*光流信息*/
    SendPack.MTF01_roll         =   control.MTF01_roll_agnle;
    SendPack.MTF01_pitch        =   control.MTF01_pitch_agnle;
    SendPack.MTF01_ToF_status   =   payload.tof_status;
    SendPack.MTF01_distance     =   payload_filtered.distance;
    SendPack.MTF01_Px           =   payload_filtered.Px;
    SendPack.MTF01_Py           =   payload_filtered.Py;
    SendPack.MTF01_Vx           =   payload_filtered.Vx;
    SendPack.MTF01_Vy           =   payload_filtered.Vy;

    /*油门信息*/
    SendPack.PWM1   =   TIM_GetCapture2(TIM9);
    SendPack.PWM2   =   TIM_GetCapture3(TIM9);
    SendPack.PWM3   =   TIM_GetCapture4(TIM9);
    SendPack.PWM4   =   TIM_GetCapture1(TIM9);

    /*状态信息*/
    SendPack.flight_mode    = control.flight_mode;
    SendPack.control_mode   = control.CONTROL_MODE;

    /*遥控器信息*/
    for(int i=0; i < 16; i++){
        SendPack.CrsfChannels[i] = CrsfChannels[i];
    }
    Serial_send_char(USART1, (uint8_t *)&SendPack, sizeof(SendPack));
}


void Serial_send_char(USART_TypeDef *USART, uint8_t *data, int size)
{
    for(int i=0; i<size; i++){
        while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
        USART_SendData(USART, data[i]);
        printf("\r\n");
    }
}


void Graph_print()
{

    //TIM9为四路PWM输出
    printf("d: %d, %d, %d, %d, %f, %f, %f\r\n", TIM_GetCapture2(TIM9),
            TIM_GetCapture3(TIM9),
            TIM_GetCapture1(TIM9),
            TIM_GetCapture4(TIM9),
            MPU6050_para_filted.yaw,
            MPU6050_para_filted.pitch,
            MPU6050_para_filted.roll);
}

void String_print()
{

    // printf("dps310_pressure:%d\r\n",DPS310_Pressure);
    
//    return;
    // printf("MTF01_roll_agnle:%f\r\n",control.MTF01_roll_agnle);      //光流计未启用
    // printf("MTF01_pitch_agnle:%f\r\n",control.MTF01_pitch_agnle);
    // printf("statues:%d\r\n",payload.tof_status);

    // printf("compensate_factor=%f\r\n",compensate_factor);
    // printf("xPortGetMinimumEverFreeHeapSize = %d\r\n",xPortGetMinimumEverFreeHeapSize());   //剩余堆空间

    // printf("yaw_filted=%f\r\n",MPU6050_para_filted.yaw);
    // printf("pitch_filted=%f\r\n",MPU6050_para_filted.pitch);
    // printf("roll_filted=%f\r\n",MPU6050_para_filted.roll);

    // printf("av_yaw_filted=%f\r\n",MPU6050_para_filted.av_yaw);
    // printf("av_pitch_filted=%f\r\n",MPU6050_para_filted.av_pitch);
    // printf("av_roll_filted=%f\r\n",MPU6050_para_filted.av_roll);

    // printf("%f,%f,%f,%f,%f,%f\r\n",MPU6050_para_filted.yaw,MPU6050_para_filted.pitch,MPU6050_para_filted.roll,MPU6050_para_filted.av_yaw,MPU6050_para_filted.av_pitch,MPU6050_para_filted.av_roll);

    // printf("yaw=%f\r\n",MPU6050_para.yaw);
    // printf("pitch=%f\r\n",MPU6050_para.pitch);
    // printf("roll=%f\r\n",MPU6050_para.roll);
    // printf("av_yaw=%d\r\n",MPU6050_para.av_yaw);
    // printf("av_pitch=%d\r\n",MPU6050_para.av_pitch);
    // printf("av_roll=%d\r\n",MPU6050_para.av_roll);

    // printf("yaw_outer_loop=%f\r\n",control.PID_yaw_outerloop.out);
    // printf("roll_outer_loop=%f\r\n",control.PID_roll_outerloop.out);
    // printf("pitch_outer_loop=%f\r\n",control.PID_pitch_outerloop.out);

    // printf("yaw_out=%f\r\n",control.PID_yaw_innerloop.out);
    // printf("roll_out=%f\r\n",control.PID_roll_innerloop.out);
    
    // printf("pitch_out=%f\r\n",control.PID_pitch_innerloop.out);


    // printf("MOTOR_PWM1:%d\r\n",TIM_GetCapture4(TIM9));   //n=1
    // printf("MOTOR_PWM2:%d\r\n",TIM_GetCapture2(TIM9));   //n=2
    // printf("MOTOR_PWM3:%d\r\n",TIM_GetCapture3(TIM9));   //n=3
    // printf("MOTOR_PWM4:%d\r\n",TIM_GetCapture1(TIM9));   //n=4

    //这是算过PID、Mixer后的PWM输出
    // printf("MOTOR_PWM1:%d\r\n",control.PWM_Out1);   //n=1
    // printf("MOTOR_PWM2:%d\r\n",control.PWM_Out2);   //n=2
    // printf("MOTOR_PWM3:%d\r\n",control.PWM_Out3);   //n=3
    // printf("MOTOR_PWM4:%d\r\n",control.PWM_Out4);   //n=4  
    // printf("%f,%f,%d,%d,%d,%d\r\n",(control.Throttle/compensate_factor),control.PID_roll_innerloop.out,control.PWM_Out1,control.PWM_Out2,control.PWM_Out3,control.PWM_Out4);
    // printf("%f\r\n",control.PID_roll_innerloop.out);
    // printf("%f\r\n",MPU6050_para_filted.yaw);
    printf("%d,%d,%d,%d,%f,%f,%f,%f\r\n",control.PWM_Out1,control.PWM_Out2,control.PWM_Out3,control.PWM_Out4,control.Yaw,control.Pitch,control.Roll,control.Throttle);

    // printf("MOTOR_PWM1:%d\r\n",TIM_GetCapture1(TIM9));   //n=1
    // printf("MOTOR_PWM2:%d\r\n",TIM_GetCapture2(TIM9));   //n=2
    // printf("MOTOR_PWM3:%d\r\n",TIM_GetCapture3(TIM9));   //n=3
    // printf("MOTOR_PWM4:%d\r\n",TIM_GetCapture4(TIM9));   //n=4  

    // printf("distance:%d\r\n",payload_filtered.distance);
    // printf("Px:%f\r\n",payload_filtered.Px);
    // printf("Py:%f\r\n",payload_filtered.Py);
    // printf("Vx:%f\r\n",payload_filtered.Vx);
    // printf("Vy:%f\r\n",payload_filtered.Vy);

    // if(control.flight_mode==GPS){
    //     printf("flight_mode:GPS\r\n");
    // }
    // else if(control.flight_mode==Stable){
    //     printf("flight_mode:Stable\r\n");
    // }
    // else{
    //     printf("flight_mode:Free\r\n");
    // }

    // if(control.is_locked == Unlocked){
    //     printf("Throttle Unlocked\r\n");
    // }
    // else{
    //     printf("Throttle Locked\r\n");
    // }

    // if(control.CONTROL_MODE == PID_CONTROL_MODE){
    //     printf("Control_mode:PID\r\n\n");
    // }else if(control.CONTROL_MODE == RAW_CONTROL_MODE){
    //     printf("Control_mode:RAW\r\n\n");
    // }else if(control.CONTROL_MODE == STABLE_CONTROL_MODE){
    //     printf("Control_mode:Stable\r\n\n");
    // }

    // printf("Control.Throttle:%d\r\n",control.Throttle);

//    printf("ch1:ELRS_roll %d\r\n",CrsfChannels[0]);    // ELRS_roll
//    printf("ch2:ELRS_Pitch %d\r\n",CrsfChannels[1]);    // ELRS_Pitch
//    printf("ch3:ELRS_Throttle %d\r\n",CrsfChannels[2]);    // ELRS_Throttle
//    printf("ch4:ELRS_yaw %d\r\n",CrsfChannels[3]);    // ELRS_yaw
//    printf("ch5:ELRS_Throttle_lock %d\r\n",CrsfChannels[4]);    // ELRS_Throttle_lock
//    printf("ch6:ELRS_Control_mode %d\r\n",CrsfChannels[5]);    // ELRS_Control_mode
//    printf("ch7:ELRS_mode %d\r\n",CrsfChannels[6]);    // ELRS_mode


}

