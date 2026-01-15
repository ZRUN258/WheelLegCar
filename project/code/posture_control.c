#include "posture_control.h"

uint32 system_count = 0;//系统计数器
cascade_common_value_struct my_cascade_common_value;
bool run_flag = true;
static pid_cycle_struct angle_speed_cycle = {//角速度闭环控制结构体初始化
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_value = 0.0f,
    .i_value_max = 0.0f,
    .out = 0.0f,
    .p_value_last = 0.0f
};
static pid_cycle_struct my_angle_cycle = {//角度闭环控制结构体初始化
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_value = 0.0f,
    .i_value_max = 0.0f,
    .out = 0.0f,
    .p_value_last = 0.0f
};
void imu_data_get(void)
{
    // 获取IMU数据的函数实现
    imu660ra_get_acc();
    imu660ra_get_gyro();
    
    imu660ra_gyro_x = imu660ra_gyro_x - 4;
    imu660ra_gyro_y = imu660ra_gyro_y + 6;
    //imu660ra_gyro_z = imu660ra_gyro_z + 6;

    if(func_abs(imu660ra_gyro_x) <= 5)
    {
        imu660ra_gyro_x = 0;
    }
    if(func_abs(imu660ra_gyro_y) <= 5)
    {
        imu660ra_gyro_y = 0;
    }
    if(func_abs(imu660ra_gyro_z) <= 5)
    {
        imu660ra_gyro_z = 0;
    }
}

void dynamic_motor_control(void)
{
    int16 left_motor_duty;
    int16 right_motor_duty;
    if(run_flag){
        if(my_cascade_common_value.common_value > 2000 || my_cascade_common_value.common_value < -2000){
            small_driver_set_duty(0,0);
            run_flag = false;
        }
        left_motor_duty = func_limit_ab(angle_speed_cycle.out, -10000, 10000);
        right_motor_duty = func_limit_ab(angle_speed_cycle.out, -10000, 10000);
        small_driver_set_duty(left_motor_duty,right_motor_duty);
    }
    else{
        small_driver_set_duty(0,0);
    }      
}

void pit_isr_callback(void)
{
    // PIT中断回调函数的实现
    system_count++;
    imu_data_get();
    if(system_count % 5 ==0){
        //first_order_complementary_filter
        first_order_complementary_filter(&my_cascade_common_value,*(my_cascade_common_value.gyro_raw_data_l),*(my_cascade_common_value.acc_raw_data_l));
        pid_control_pd(&my_angle_cycle,0.0f,my_cascade_common_value.common_value);
    }
    pid_control_pd(&angle_speed_cycle,-my_angle_cycle.out,*(my_cascade_common_value.gyro_raw_data_l));
    dynamic_motor_control();
}

//*********一阶互补滤波*************

void first_order_complementary_filter_init(void){
    my_cascade_common_value.gyro_raw_data_l = &imu660ra_gyro_x;
    my_cascade_common_value.acc_raw_data_l = &imu660ra_acc_y;
    my_cascade_common_value.gyro_ration = -4;   // 角速度置信度
    my_cascade_common_value.acc_ration = 4;    // 加速度置信度
    my_cascade_common_value.common_value = 0;
    my_cascade_common_value.dt = 0.005f;          // 采样时间间隔
    my_cascade_common_value.mechanical_offset = 1200; //机械偏置:小车在-800左右为平衡点
}


void first_order_complementary_filter(cascade_common_value_struct* filter,int16 gyro_raw_data,int16 acc_raw_data){
    //一阶互补滤波算法实现
    float gyro_temp;
    float acc_temp;

    gyro_temp = gyro_raw_data * filter->gyro_ration; //角速度*角速度置信度
    acc_temp = (acc_raw_data-filter->temp_value) * filter->acc_ration;    //加速度*加速度置信度gyro
    filter->temp_value += ((gyro_temp + acc_temp) * filter->dt); //互补滤波计算
    filter->common_value = filter->temp_value + filter->mechanical_offset; //加上机械偏置
}


//**********PID控制器-PD************
void pid_control_pd(pid_cycle_struct* pid_cycle,float target_value,float current_value){
    float error;
    float p_value;
    float d_value;
    
    // 计算误差
    error = target_value - current_value;
    
    // 计算比例项
    p_value = pid_cycle->kp * error;
    
    // 计算微分项（基于比例项的变化）
    d_value = pid_cycle->kd * (p_value - pid_cycle->p_value_last);
    
    // 保存当前比例项用于下次微分计算
    pid_cycle->p_value_last = p_value;
    
    // 计算输出（PD控制器只有P和D项，没有I项）
    pid_cycle->out = p_value + d_value;
}
