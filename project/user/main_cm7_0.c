/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技这是一个模板这是一个模板这是一个模板这是一个模板
*这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板
* 本文件是 CYT4BB 开源库的一部分这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板
*这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板
* CYT4BB 开源库 是免费软件这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板这是一个模板
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各设

// **************************** 变量定义区域 ****************************
#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)



int main(void){
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
