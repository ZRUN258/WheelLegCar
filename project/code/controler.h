/*
    控制器模块，包含按键、LED和无线串口调参功能
    - 按键初始化与状态检测
    - LED 初始化与控制
    - 无线串口调参功能实现
    ⚠️ 有依赖问题，不能将头文件加到 zf_common_headfile.h 中
*/
#ifndef CONTROLER_H_
#define CONTROLER_H_

#include "zf_common_headfile.h"
#include "posture_control.h"

#define OPTIMIZER_UART                       (UART_1)
#define OPTIMIZER_BAUDRATE                   (115200)        //蓝牙-串口模块波特率为115200
#define OPTIMIZER_RX                         (UART1_RX_P04_0)
#define OPTIMIZER_TX                         (UART1_TX_P04_1)

#define WIRELESS_SPI_INDEX              (SPI_0)                     // 定义使用的SPI号
#define WIRELESS_SPI_SPEED              (10 * 1000 * 1000)          // 硬件 SPI 速率
#define WIRELESS_SPI_SCK_PIN            (SPI0_CLK_P02_2 )           // 定义SPI_SCK引脚
#define WIRELESS_SPI_MOSI_PIN           (SPI0_MOSI_P02_1)           // 定义SPI_MOSI引脚
#define WIRELESS_SPI_MISO_PIN           (SPI0_MISO_P02_0)           // 定义SPI_MISO引脚  IPS没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
#define WIRELESS_SPI_CS_PIN             (P02_3)                     // 定义SPI_CS引脚 采用软件CS引脚
#define WIRELESS_SPI_INT_PIN            (P02_4)                     // 定义握手引脚
#define WIRELESS_SPI_RST_PIN            (P23_0)                     // 定义复位引脚

#define BUTTON1                    (P20_0)
#define BUTTON2                    (P20_1)
#define BUTTON3                    (P20_2)
#define BUTTON4                    (P20_3)

#define LED                     (P19_0)

typedef enum {
    bt1 = 0,
    bt2,
    bt3,
    bt4
} Button;

typedef enum {
    on = 0,
    off,
    toggle
} LedCmd;

void button_init(void);
bool button_press(Button bt);
void led_init(void);
void led(LedCmd cmd);
void serial_optimizer_callback(cascade_value_struct* cascade_value_ptr); //接收串口信息，调整参数
void serial_optimizer_init(void); //串口初始化
void air_printf(const char* fmt, ...); //无线串口打印函数
void wireless_spi_init(void);
uint32 wireless_spi_send_buffer(const uint8 *buff, uint32 len);
uint32 wireless_spi_read_buffer(uint8 *buff, uint32 len);
#endif