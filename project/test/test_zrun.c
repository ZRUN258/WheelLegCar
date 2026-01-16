#include "test_zrun.h"
#include "controler.h"

// **************************** 变量定义区域 ****************************
#define SERVO_MOTOR_PWM1            (TCPWM_CH13_P00_3)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM2            (TCPWM_CH12_P01_0)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM3            (TCPWM_CH11_P01_1)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM4            (TCPWM_CH20_P08_1)                          // 主板上的有刷电机2引脚飞线
#define SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (150)                                       // 定义主板上舵机活动范围 角度

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif

float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

void zrun_test_controler(void){
    led_init();
    cascade_init();
    serial_optimizer_init(); //串口初始化
    while(true){
        printf("mechanical_offset:%d\r\n", cascade_value.cascade_common_value.mechanical_offset);
        printf("angular_speed_cycle.kp:%f\r\n", cascade_value.angular_speed_cycle.kp);
        printf("angular_speed_cycle.kd:%f\r\n", cascade_value.angular_speed_cycle.kd);
        printf("angle_cycle.kp:%f\r\n", cascade_value.angle_cycle.kp);
        printf("angle_cycle.kd:%f\r\n", cascade_value.angle_cycle.kd);
        system_delay_ms(1000);
    }
}
void zrun_test_servo(void){
    button_init();
    while(true){
        if(button_press(bt1)){
            break;
        }
        system_delay_ms(10);
    }
    //pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, 300);
    //pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, 300);
    pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, 300);
    while(true)
    {
        // 此处编写需要循环执行的代码

        if(servo_motor_dir)
        {
            servo_motor_duty ++;
            if(servo_motor_duty >= SERVO_MOTOR_R_MAX)
            {
                servo_motor_dir = 0x00;
            }
        }
        else
        {
            servo_motor_duty --;
            if(servo_motor_duty <= SERVO_MOTOR_L_MAX)
            {
                servo_motor_dir = 0x01;
            }
        }
        
        //pwm_set_duty(SERVO_MOTOR_PWM1, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty));
        //pwm_set_duty(SERVO_MOTOR_PWM2, (uint16)SERVO_MOTOR_DUTY(200-servo_motor_duty));
        pwm_set_duty(SERVO_MOTOR_PWM4, (uint16)SERVO_MOTOR_DUTY(200-servo_motor_duty));
        system_delay_ms(15);                                                   // 延时
      
      
      
        // 此处编写需要循环执行的代码a
    }
}
void zrun_test_led(void){
  led_init();
  while(true){
    led(on);
    system_delay_ms(1000);
    led(off);
    system_delay_ms(1000);
}
}
void zrun_test_balance(void)
{
    //clock_init(SYSTEM_CLOCK_250M);                              // 时钟初始化
    //debug_init();                                               // debug 串口初始化
    button_init();
    motor_control_init();               // 电机控制初始化
    cascade_init();                     // 滤波链初始化

    while(1)
    {
        if(imu660ra_init())
        {
           printf("\r\n imu660ra init error.");        // imu660ra 初始化失败
        }
        else
        {
           break;
        }
    }
    pit_ms_init( PIT_CH10, 1 );          // pit通道10初始化，周期中断时间1ms
    while(true){
        printf("%d,%d,%d,%d,%d,%d,%f\r\n", imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z,cascade_value.cascade_common_value.filtered_value);
        //printf("%d,%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        if(button_press(bt1)){
            run_flag = true;
            printf("bt1\r\n");
        }
        else if(button_press(bt2)){
            run_flag = false;
            printf("bt2\r\n");
        }       
        system_delay_ms(10); 

    }
}