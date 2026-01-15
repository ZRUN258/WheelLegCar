#include "test.h"

// **************************** 变量定义区域 ****************************
#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)
#define LED1                    (P10_0)

void zrun_test_balance(void)
{
    clock_init(SYSTEM_CLOCK_250M);                     // 时钟初始化
    debug_init();                     // debug 串口初始化
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY4 输入 默认高电平 上拉输入
    
    motor_control_init();            // 电机控制初始化
    first_order_complementary_filter_init(); //一阶互补滤波初始化

    while(1)
    {
        if(imu660ra_init())
        {
           printf("\r\n imu660ra init error.");                                 // imu660ra 初始化失败
        }
        else
        {
           break;
        }
    }
    pit_ms_init( PIT_CH10, 1 );          // pit通道10初始化，周期中断时间10ms
    while(true){
        printf("%d,%d,%d,%d,%d,%d,%f\r\n", imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z,my_cascade_common_value.common_value);
        //printf("%d,%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        if(!gpio_get_level(KEY1)){
            //small_driver_set_duty(10,10);      // 无刷驱动 设置电机占空比
            run_flag = true;
            printf("key1\r\n");
        }
        else if(!gpio_get_level(KEY2)){
            //small_driver_set_duty(-10,-10);    // 无刷驱动 设置电机占空比
            run_flag = false;
            printf("key2\r\n");
        }
        // else if(!gpio_get_level(KEY3)){
        //     small_driver_set_duty(500,500);        // 无刷驱动 设置电机占空比
        //     printf("key3\r\n");
        // }
        // else if (!gpio_get_level(KEY4)){
        //     small_driver_set_duty(-500,-500);      // 无刷驱动 设置电机占空比
        //     printf("key4\r\n");
        // }
        // else small_driver_set_duty(0,0);
        
        system_delay_ms(10); 
        //small_driver_get_speed();

    }
}