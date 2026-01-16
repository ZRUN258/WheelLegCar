#include "controler.h"
#include "posture_control.h"
#include <string.h>

void button_init(void){
    gpio_init(bt1, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(bt2, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(bt3, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(bt4, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY4 输入 默认高电平 上拉输入
}

void led_init(void){
    gpio_init(LED, GPO, GPIO_HIGH, GPO_PUSH_PULL);              // 初始化 LED 输出 默认高电平 推挽输出
}

bool button_press(Button bt){
    switch(bt){
        case bt1:
            return !gpio_get_level(BUTTON1);                       // 按键按下返回 true
        case bt2:
            return !gpio_get_level(BUTTON2);                       // 按键按下返回 true
        case bt3:
            return !gpio_get_level(BUTTON3);                       // 按键按下返回 true
        case bt4:
            return !gpio_get_level(BUTTON4);                       // 按键按下返回 true
        default:
            return false;
    }
}

void led(LedCmd cmd){
    switch(cmd){
        case on:
            gpio_set_level(LED, 0);                             // LED 灯打开
            break;
        case off:
            gpio_set_level(LED, 1);                             // LED 灯关闭
            break;
        case toggle:
            gpio_set_level(LED, !gpio_get_level(LED));          // LED 灯取反
            break;
        default:
            break;
    }
}

void serial_optimizer_init(void){
    uart_init(OPTIMIZER_UART, OPTIMIZER_BAUDRATE, OPTIMIZER_RX, OPTIMIZER_TX);      // 串口初始化
    
    uart_rx_interrupt(OPTIMIZER_UART, 1);                                                    // 使能串口接收中断
}

void serial_optimizer_callback(cascade_value_struct* cascade_value_ptr){ //接收串口信息，调整参数
    // 帧格式（8字节）：[0]=0x55, [1]=0x20, [2]=channel, [3]=reserved, [4..7]=little-endian float
    static uint8 rx_buffer[8] = {0};
    static uint8 rx_index = 0;
    uint8 receive_data;

    if(uart_query_byte(OPTIMIZER_UART, &receive_data)) // 接收串口数据（逐字节）
    {
        // 帧头重同步：收到0x55且当前缓冲不是以0x55开头，则重置
        if(receive_data == 0x55 && rx_buffer[0] != 0x55)
        {
            rx_index = 0;
        }

        rx_buffer[rx_index++] = receive_data; // 保存串口数据

        // 收满8字节后处理
        if(rx_index >= 8)
        {
            // 检查帧头和命令字
            if((rx_buffer[0] == 0x55) && (rx_buffer[1] == 0x20))
            {
                uint8 channel = rx_buffer[2];
                float f;
                memcpy(&f, &rx_buffer[4], sizeof(float)); // 小端浮点

                switch(channel)
                {
                    case 0x01: // 通道1：机械偏置
                        cascade_value_ptr->cascade_common_value.mechanical_offset = (int16)f;
                        break;
                    case 0x02: // 通道2：角速度闭环 kp
                        cascade_value_ptr->angular_speed_cycle.kp = f;
                        break;
                    case 0x03: // 通道3：角速度闭环 kd
                        cascade_value_ptr->angular_speed_cycle.kd = f;
                        break;
                    case 0x04: // 通道4：角度闭环 kp
                        cascade_value_ptr->angle_cycle.kp = f;
                        break;
                    case 0x05: // 通道5：角度闭环 kd
                        cascade_value_ptr->angle_cycle.kd = f;
                        break;
                    default:
                        // 其他通道暂不处理
                        break;
                }
            }

            // 重置准备接收下一帧
            rx_index = 0;
            memset(rx_buffer, 0, 8);
        }
    }
}