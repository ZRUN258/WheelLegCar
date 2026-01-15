#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 中断通道占用说明：
/*
pit_chan10      zrun_test_balance

*/


int main(void){
    clock_init(SYSTEM_CLOCK_250M);                     // 时钟初始化
    debug_init();                                      // debug 串口初始化
    //测试函数
    zrun_test_led();
    //zrun_test_balance();
    return 0;
}
