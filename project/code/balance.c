/*
 * balance.c
 *
 *  Created on: 2025年3月13日
 *
 */
#include "zf_common_headfile.h"


/***************************************************************************.
 * 函数名称：balance_control_PID(1.0 , 0)
 * 函数功能：
 * 参数说明：*
 * 函数返回：
 * 备注：*
 **************************************************************************/
void balance_control_PID(void)
{
    sys_count++;

//=============================================================================================================//
//============================20ms,转向环，速度环，差速度环============================================================//
//=============================================================================================================//
    if(sys_count%10 == 0)
    {



        //=========转向环    ========//
        Turn_Control();
        //=========转向环滤波========//
        static float camERR_out_temp = 0.0f;
        camERR.out = low_pass(0.4 , camERR.out , &camERR_out_temp);//0.1


        //=========转向环    ========//
        float cam_out = Turn_Control_inv();
        //=========转向环滤波========//
        static float cam_out_temp = 0.0f;
        cam_out = low_pass(0.4 , cam_out , &cam_out_temp);//0。8

        //=========横断转向控制========//
        jump_control_replaced();


        //=======输出目标角速度======//
        float weight_param = 0.0;//对逆透视图像转向输出的置信权重
        if (imageState.Ramp_Flag || imageState.jump_Find || imageState.Island_State){
            weight_param = 0.0;
            cam_out_temp = 0;cam_out = 0;
        }
        else{
            weight_param = 0.3;
        }
        gyro_z_target = cam_out * weight_param + camERR.out * ( 1.0 - weight_param);
//        gyro_z_target = 0;

        //=========速度环=======//
        Speed_Control();

    }
//=============================================================================================================//
//============================5ms,角度环,横滚环====================================================================//
//=============================================================================================================//
    if(sys_count%5 == 0)
    {
        //=====元素处理与控制逻辑======//
        element_process();

        //=========腿部控制=======//
        Leg_resolve_control_xy(leg_x + speed_steer_cycle.output * speed_steer_way, leg_y , 0.5*EH , 0.0f , 180);

        //========角度环=========//
        pid_controler(machine_zero , imu_AHRS.pitch  , &angle_cycle);
        printf("%f\n" , imu_AHRS.pitch);
}

//=============================================================================================================//
//============================1ms,角速度环,舵机执行，电机执行==========================================================//
//=============================================================================================================//
    if(sys_count%1 == 0)
    {
        //========角速度环=======//
        pid_controler( angle_cycle.output , imu_AHRS.gy_used , &angular_speed_cycle);


        //=========转向角速度环========//
        pid_controler(   gyro_z_target  , -imu_AHRS.gz_used   , &speed_yaw_cycle );//z轴左转为负，右转为正


        //=========转向角速度环滤波========//
        static float speed_yaw_cycle_temp = 0.0f;
        speed_yaw_cycle.output = low_pass(0.2 , speed_yaw_cycle.output , &speed_yaw_cycle_temp);//0.2


        //=========转向速度补偿============//
        float L_wheel = 0.165;
        float v_sub_target = -imu_AHRS.gyro_z * L_wheel;
        float v_sub = (Motor_speed_left_raw - Motor_speed_right_raw) * v2x_real;
        float v_add1 = (Motor_speed_left_raw + Motor_speed_right_raw)/2 * v2x_real;
        float v_add2 = (Motor_speed_left_raw + Motor_speed_right_raw)/2 * v2x;
//        printf("%f,%f,%f\n" ,v  , v_add1 , v_add2);
        pid_controler(   v_sub_target  , v_sub   , &v_sub_cycle );//z轴左转为负，右转为正

        //=====电机输出+舵机输出====//
        dynamic_servo_control();
        dynamic_motor_control();


    }
}





/***************************************************************************.
 * 函数名称：Turn_Control()
 * 函数功能：
 * 参数说明：*
 * 函数返回：
 * 备注：*
 **************************************************************************/
void Turn_Control(void)
{
    camERR.last_err = camERR.err ;

    camERR.err =  camERR.finalCenterError[0] * 0.5f + camERR.finalCenterError[1] * 0.25f + camERR.finalCenterError[2] * 0.25f;

    camERR.kp_out       = camERR.err * camERR.kp;
    camERR.kp_sq_out    = fabs(camERR.err) * camERR.err * camERR.kp_sq;

    static float last_dout;
    last_dout           = camERR.kd_out;
    camERR.kd_out       = (camERR.err - camERR.last_err)* camERR.kd;
    camERR.kd_out       = camERR.kd_out * camERR.low_pass + (1 - camERR.low_pass) * last_dout;

    camERR.kd_gyro_out  = imu660ra_gyro_z * camERR.kd_gyro;

    camERR.out = camERR.kp_out + camERR.kp_sq_out + camERR.kd_out + camERR.kd_gyro_out;



}


/***************************************************************************.
 * 函数名称：Turn_Control_inv()
 * 函数功能：
 * 参数说明：*
 * 函数返回：
 * 备注：*
 **************************************************************************/
float Turn_Control_inv(void)
{
    float v_real = ( v / v2x * v2x_real ) / 2;

    float target_w = v_real/radius;

    pid_controler(0.0 , target_w , &turn_inv_cycle);

//    printf("w = %f\n",target_w);

    return -turn_inv_cycle.output;

}

/***************************************************************************.
 * 函数名称：Speed_Control()
 * 函数功能：
 * 参数说明：*
 * 函数返回：
 * 备注：*
 **************************************************************************/

 void anti_windup(void)
 {
   //抗饱和积分
     if(fabs(speed_steer_cycle.err) > 0.3){
         speed_cycle.ki = 0.0f;
         speed_steer_cycle.ki = 0.0f;
     }
     else{
         speed_steer_cycle.ki = 0.005f;
         speed_cycle.ki = 0.02f;
     }
 }


float anti_windup_speed_A = 0.6;
float anti_windup_speed_B = 0.6;

void Speed_Control(void)
{
    //===========速度规划=====================//
    static float v_target_temp = 0.0f;
    static float v_target_last = 0.0f;

    v_target = v_max_temp - fabs(gyro_z_target) * speed_param/2 - fabs(imu_AHRS.gz_used) * speed_param/2  ;
//    v_target = v_max_temp + fabs(gyro_z_target - imu_AHRS.gz_used) * speed_param  ;

//    v_target = clip(v_target , v_target_last - 0.1 , v_target_last + 0.1);

    if(imageState.Single_Bridge_Flag)
        v_target = low_pass(0.8 , v_target , &v_target_temp);
    else
        v_target = low_pass(0.1 , v_target , &v_target_temp);

    v_target_last = v_target;

//    printf("%f\n",v_target);
    //===========速度数据处理============//
    static float v_hat_last = 0.0f;
    float alpha = 0.3;
    LIMIT(v_hat,v_hat_last-0.1,v_hat_last+0.1);
    v = v_hat * alpha + (1.0 - alpha) * v;
//    printf("%f,%f,%d,%d\n",v,(Motor_speed_left_raw + Motor_speed_right_raw) * v2x,Motor_speed_left_raw,Motor_speed_right_raw);

    //=============速度环==============//
    //转换到角度——速度环
    if(speed_way!=0)
    {
//        pid_controler(v_target   , v    , &speed_cycle );
        test_pid_controler(v_target   , v    , &speed_cycle , anti_windup_speed_A,anti_windup_speed_B);
    }
    //转换到舵机——速度环
    if(speed_steer_way!=0)
    {
//        pid_controler(v_target   ,  v  , &speed_steer_cycle );
        test_pid_controler(v_target   ,  v  , &speed_steer_cycle ,anti_windup_speed_A,anti_windup_speed_B);
    }

    FuzzyPID_controller( v_target ,  v , &fpid_speed);

    //================================//
    v_hat_last = v_hat;
}


/***************************************************************************.
 * 函数名称：void dynamic_motor_control(int16 LO , int16 RO)
 * 函数功能：
 * 参数说明：*
 * 函数返回：
 * 备注：*
 **************************************************************************/
void dynamic_motor_control(void)
{
    int16 LO,RO;

    LO = (int16)(angular_speed_cycle.output + speed_yaw_cycle.output + v_sub_cycle.output);
    RO = (int16)(angular_speed_cycle.output - speed_yaw_cycle.output - v_sub_cycle.output);

    LIMIT(LO,-10000,10000);
    LIMIT(RO,-10000,10000);


    //===电机启动控制（遥控---按键---串口---姿态保护---出界保护---斑马线）
    if(motor_run==1){
        left_motor_duty  =  LO;
        right_motor_duty =  RO;
    }
    else{
        left_motor_duty  = 0;
        right_motor_duty = 0;
        imageState.Garage_Find=0;
    }

    small_driver_set_duty(right_motor_duty,-left_motor_duty);
}



float low_pass(float alpha , float data , float* temp)
{
    float output;
    output = alpha * data + (1 - alpha) * (* temp);
    *temp = output;
    return output;
}






























































/***************************************************************************.
 * 函数名称：Leg_Control()
 * 函数功能：
 * 参数说明：*
 * 函数返回：
 * 备注：*
 **************************************************************************/
void Leg_Control(void)
{
//=======================单边桥识别后的腿部控制---低腿方法====================================//
//    printf("%d,%f,%f\n",imageState.Single_Bridge_Cnt,L_ditance,v_max);

    if(imageState.Single_Bridge_Flag == 1 )
    {
        //参数变换
        speed_cycle.kd = 200.0f;


        //抗饱和积分
        if(fabs(speed_steer_cycle.err) > 0.3){
            speed_cycle.ki = 0.0f;
            speed_steer_cycle.ki = 0.0f;
        }
        else{
            speed_steer_cycle.ki = 0.005f;
            speed_cycle.ki = 0.02f;
        }


        if(/*460*/500<=imageState.Single_Bridge_Cnt && imageState.Single_Bridge_Cnt<=500){
            v_control = 1.0f;
//            steer_theta=(v_target-v)*2.2*15.0;
            EH=0.0;
        }

        else if(/*430*/470<=imageState.Single_Bridge_Cnt && imageState.Single_Bridge_Cnt<=500/*460*/){
            v_control = 1.0f;
            L_ditance = 44.0 + 1.0335*(500-imageState.Single_Bridge_Cnt);
            steer_theta=(v_target-v)*2.2*4.0;
//            steer_theta=0.0f;
            EH=0.0;
            if(imageState.Single_Bridge_Cnt == 470)single_bridge_param_change(1);
        }

        else if(imageState.Single_Bridge_Cnt <= 470){
            steer_theta=0.0f;
            EH=roll_low_pass(imu_EKF.roll,0.01)*roll_leg;
        }



        if(imageState.Single_Bridge_Cnt>0){
            imageState.Single_Bridge_Cnt--;
            if(imageState.Single_Bridge_Cnt==0){
                imageState.Single_Bridge_Flag = 0;
                single_bridge_param_change(0);
            }
        }





    }
//=======================坡道识别后的腿部控制====================================//
    else if(imageState.Ramp_Flag )
    {
        steer_theta=(v_target-v)*2.2*6.0;
    }
//=======================正常巡线的腿部控制====================================//
    else{
//        leg_x = 0;
        leg_y = 44;
        L_ditance=44;//75
        EH=gyroz_target_low_pass(gyro_z_target,0.1)*4.0;
        steer_theta=(v_target-v)*2.2*0.0;
    }
}


/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     死区补偿
// 参数说明     *input_L    左电机
                *input_R    右电机
// 返回参数     null
// 使用示例     dead_compensate(&input_L, &input_R);
// 备注信息     LQR函数中调用
-------------------------------------------------------------------------------------------------------------------*/
void dead_compensate(int16 *input_L, int16 *input_R)
{
    if(*input_L > 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_correct, 10000);
    }
    else if(*input_L < 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_negative, 10000);
    }
    else
    {
        *input_L = 0;
    }
    if(*input_R > 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_correct, 10000);
    }
    else if(*input_R < 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_negative, 10000);
    }
    else
    {
        *input_R = 0;
    }
}


/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度低通滤波
// 参数说明

// 返回参数     null
// 使用示例
// 备注信息     函数中调用
-------------------------------------------------------------------------------------------------------------------*/
float angle_low_pass(float angle , float alpha)
{
    static float last_output = 0.0f;
    last_output = angle * alpha + (last_output) * (1 - alpha);
    return last_output;
}
/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度低通滤波
// 参数说明

// 返回参数     null
// 使用示例
// 备注信息     函数中调用
-------------------------------------------------------------------------------------------------------------------*/
float roll_low_pass(float angle , float alpha)
{
    static float roll_last_output = 0.0f;
    roll_last_output = angle * alpha + (roll_last_output) * (1 - alpha);
    return roll_last_output;
}
/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度低通滤波
// 参数说明

// 返回参数     null
// 使用示例
// 备注信息     函数中调用
-------------------------------------------------------------------------------------------------------------------*/
float gyroz_target_low_pass(float angle , float alpha)
{
    static float gyroz_target_last_output = 0.0f;
    gyroz_target_last_output = angle * alpha + (gyroz_target_last_output) * (1 - alpha);
    return gyroz_target_last_output;
}

#define LOOP_WINDOW_SIZE_gyroz_target 50
float sliding_window_filter_gyroz_target(float new_value)
{
    static float window_gyroz_target[LOOP_WINDOW_SIZE_gyroz_target];  // 环形缓冲区
    static int index_gyroz_target = 0;              // 当前写入位置
    static int count_gyroz_target = 0;              // 当前有效数据量
    static float sum_gyroz_target = 0.0f;           // 窗口数据累加和

    /* 更新窗口数据 */
    if (count_gyroz_target < LOOP_WINDOW_SIZE_gyroz_target) {         // 窗口未满阶段
        sum_gyroz_target += new_value;              // 直接累加新值
        window_gyroz_target[index_gyroz_target++] = new_value;   // 存储新值
        count_gyroz_target++;                       // 增加有效计数
    } else {                           // 窗口已满阶段
        sum_gyroz_target += new_value - window_gyroz_target[index_gyroz_target];  // 更新和：减去旧值，加上新值
        window_gyroz_target[index_gyroz_target++] = new_value;       // 覆盖最旧数据
    }

    /* 环形索引处理 (避免分支判断) */
    index_gyroz_target %= LOOP_WINDOW_SIZE_gyroz_target;  // 等价 if(index >= WINDOW_SIZE) index=0;

    /* 返回平均值 (自动处理未满窗口) */
    return sum_gyroz_target / count_gyroz_target;     // count取值范围：[1, WINDOW_SIZE]
}




float yaw_control(void)
{
    static float yaw_sum = 0;
    static int yaw_cnt = 0;
    float last_Line_yaw = 0.0f;

    yaw_sum+=imu_EKF.yaw;
    yaw_cnt++;
    last_Line_yaw = yaw_sum/yaw_cnt;

    return last_Line_yaw;
}
