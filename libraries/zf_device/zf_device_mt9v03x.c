/*********************************************************************************************************************
* MM32F527X-E9P Opensource Library 鍗筹紙MM32F527X-E9P 寮€婧愬簱锛夋槸涓€涓熀浜庡畼鏂?SDK 鎺ュ彛鐨勭涓夋柟寮€婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰绉戞妧
* 
* 鏈枃浠舵槸 MM32F527X-E9P 寮€婧愬簱鐨勪竴閮ㄥ垎
* 
* MM32F527X-E9P 寮€婧愬簱 鏄厤璐硅蒋浠?
* 鎮ㄥ彲浠ユ牴鎹嚜鐢辫蒋浠跺熀閲戜細鍙戝竷鐨?GPL锛圙NU General Public License锛屽嵆 GNU閫氱敤鍏叡璁稿彲璇侊級鐨勬潯娆?
* 鍗?GPL 鐨勭3鐗堬紙鍗?GPL3.0锛夋垨锛堟偍閫夋嫨鐨勶級浠讳綍鍚庢潵鐨勭増鏈紝閲嶆柊鍙戝竷鍜?鎴栦慨鏀瑰畠
* 
* 鏈紑婧愬簱鐨勫彂甯冩槸甯屾湜瀹冭兘鍙戞尌浣滅敤锛屼絾骞舵湭瀵瑰叾浣滀换浣曠殑淇濊瘉
* 鐢氳嚦娌℃湁闅愬惈鐨勯€傞攢鎬ф垨閫傚悎鐗瑰畾鐢ㄩ€旂殑淇濊瘉
* 鏇村缁嗚妭璇峰弬瑙?GPL
* 
* 鎮ㄥ簲璇ュ湪鏀跺埌鏈紑婧愬簱鐨勫悓鏃舵敹鍒颁竴浠?GPL 鐨勫壇鏈?
* 濡傛灉娌℃湁锛岃鍙傞槄<https://www.gnu.org/licenses/>
* 
* 棰濆娉ㄦ槑锛?
* 鏈紑婧愬簱浣跨敤 GPL3.0 寮€婧愯鍙瘉鍗忚 浠ヤ笂璁稿彲鐢虫槑涓鸿瘧鏂囩増鏈?
* 璁稿彲鐢虫槑鑻辨枃鐗堝湪 libraries/doc 鏂囦欢澶逛笅鐨?GPL3_permission_statement.txt 鏂囦欢涓?
* 璁稿彲璇佸壇鏈湪 libraries 鏂囦欢澶逛笅 鍗宠鏂囦欢澶逛笅鐨?LICENSE 鏂囦欢
* 娆㈣繋鍚勪綅浣跨敤骞朵紶鎾湰绋嬪簭 浣嗕慨鏀瑰唴瀹规椂蹇呴』淇濈暀閫愰绉戞妧鐨勭増鏉冨０鏄庯紙鍗虫湰澹版槑锛?
* 
* 鏂囦欢鍚嶇О          zf_device_mt9v03x
* 鍏徃鍚嶇О          鎴愰兘閫愰绉戞妧鏈夐檺鍏徃
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮€鍙戠幆澧?         IAR 9.40.1
* 閫傜敤骞冲彴          CYT2BL3
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
* 
* 淇敼璁板綍
* 鏃ユ湡              浣滆€?               澶囨敞
* 2024-11-19       pudding            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 鎺ョ嚎瀹氫箟锛?
*                   ------------------------------------
*                   妯″潡绠¤剼            鍗曠墖鏈虹鑴?
*                   TXD                 鏌ョ湅 zf_device_mt9v03x.h 涓?MT9V03X_COF_UART_TX 瀹忓畾涔?
*                   RXD                 鏌ョ湅 zf_device_mt9v03x.h 涓?MT9V03X_COF_UART_RX 瀹忓畾涔?
*                   PCLK                鏌ョ湅 zf_device_mt9v03x.h 涓?MT9V03X_PCLK_PIN 瀹忓畾涔?
*                   VSY                 鏌ョ湅 zf_device_mt9v03x.h 涓?MT9V03X_VSYNC_PIN 瀹忓畾涔?
*                   D0-D7               鏌ョ湅 zf_device_mt9v03x.h 涓?MT9V03X_DATA_PIN 瀹忓畾涔?浠庤瀹氫箟寮€濮嬬殑杩炵画鍏釜寮曡剼
*                   VCC                 3.3V鐢垫簮
*                   GND                 鐢垫簮鍦?
*                   鍏朵綑寮曡剼鎮┖
*                   ------------------------------------
********************************************************************************************************************/

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_pwm.h"
#include "zf_common_interrupt.h"

#include "zf_driver_soft_iic.h"
#include "zf_device_config.h"
#include "zf_device_mt9v03x.h"

vuint8 mt9v03x_finish_flag = 0;                                                 // 涓€鍦哄浘鍍忛噰闆嗗畬鎴愭爣蹇椾綅
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];     

static uint8 perfect_proportion = 0;

#pragma location = 0x28026024                                                   // 灏嗕笅闈㈣繖涓暟缁勫畾涔夊埌鎸囧畾鐨凴AM鍦板潃
__no_init uint8  mt9v03x_image_temp[MT9V03X_H][MT9V03X_W];                      
#pragma location = 0x28006bf0
__no_init uint16 mt9v03x_h_num;
#pragma location = 0x28006bf2
__no_init uint16 mt9v03x_w_num;

void camera_finish_callback(void)
{  
    Cy_Tcpwm_Counter_ClearTC_Intr(TCPWM0_GRP0_CNT59);
    
    SCB_InvalidateDCache_by_Addr(mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);

    memcpy(mt9v03x_image[0], mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);
    
    mt9v03x_finish_flag = 1;
}

static void mt9v03x_trig_init(void)
{   
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS59, (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS59), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul, 9u); // 80Mhz鏃堕挓琚?0鍒嗛涓?Mhz
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_TCPWM0_CLOCKS59), (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul);

    cy_stc_sysint_irq_t mt9v03x_trig_irq_cfg;
    mt9v03x_trig_irq_cfg.sysIntSrc  = tcpwm_0_interrupts_59_IRQn; 
    mt9v03x_trig_irq_cfg.intIdx     = CPUIntIdx3_IRQn;
    mt9v03x_trig_irq_cfg.isEnabled  = true;
    interrupt_init(&mt9v03x_trig_irq_cfg, camera_finish_callback, 0);

    cy_stc_tcpwm_counter_config_t tcpwm_camera_config;
    memset(&tcpwm_camera_config, 0, sizeof(tcpwm_camera_config));
    tcpwm_camera_config.period             = 0x0                                ;        // pit鍛ㄦ湡璁＄畻
    tcpwm_camera_config.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1         ;
    tcpwm_camera_config.runMode            = CY_TCPWM_COUNTER_ONESHOT           ; 
    tcpwm_camera_config.countDirection     = CY_TCPWM_COUNTER_COUNT_UP          ;
    tcpwm_camera_config.compareOrCapture   = CY_TCPWM_COUNTER_MODE_COMPARE      ;
    tcpwm_camera_config.countInputMode     = CY_TCPWM_INPUT_LEVEL               ;
    tcpwm_camera_config.countInput         = 1uL                                ;
    tcpwm_camera_config.trigger0EventCfg   = CY_TCPWM_COUNTER_OVERFLOW          ;
    tcpwm_camera_config.trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW          ;
        
    Cy_Tcpwm_Counter_Init(TCPWM0_GRP0_CNT59, &tcpwm_camera_config);
    Cy_Tcpwm_Counter_Enable(TCPWM0_GRP0_CNT59);
    Cy_Tcpwm_Counter_SetTC_IntrMask(TCPWM0_GRP0_CNT59);
    Cy_Tcpwm_TriggerStart(TCPWM0_GRP0_CNT60);
}


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鍗曠嫭璁剧疆鎽勫儚澶存洕鍏夋椂闂?
// 鍙傛暟璇存槑     light           璁惧畾鏇濆厜鏃堕棿
// 杩斿洖鍙傛暟     uint8           1-澶辫触 0-鎴愬姛
// 浣跨敤绀轰緥     mt9v03x_set_exposure_time(100);                 // 璋冪敤璇ュ嚱鏁板墠璇峰厛鍒濆鍖栦覆鍙?
// 澶囨敞淇℃伅     璁剧疆鏇濆厜鏃堕棿瓒婂ぇ鍥惧儚瓒婁寒
//              鎽勫儚澶存敹鍒板悗浼氭牴鎹垎杈ㄧ巼鍙奆PS璁＄畻鏈€澶ф洕鍏夋椂闂村鏋滆缃殑鏁版嵁杩囧ぇ
//              閭ｄ箞鎽勫儚澶村皢浼氳缃繖涓渶澶у€?
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_set_exposure_time (uint16 light)
{
    uint8   return_state        = 0;
    
    return_state = mt9v03x_sccb_set_exposure_time(light);

    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    MT9V03X SCCB 鍒濆鍖?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           0-鎴愬姛 x-澶辫触
// 浣跨敤绀轰緥     mt9v03x_sccb_init();
// 澶囨敞淇℃伅     
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_sccb_init (void)
{
    uint8 return_state  = 1;
    
    soft_iic_info_struct mt9v03x_iic_struct;
    soft_iic_init(
        &mt9v03x_iic_struct, 0,
        MT9V03X_COF_IIC_DELAY,
        MT9V03X_COF_IIC_SCL,
        MT9V03X_COF_IIC_SDA);
    
    if(!mt9v03x_sccb_check_id(&mt9v03x_iic_struct))
    {
        // 闇€瑕侀厤缃埌鎽勫儚澶寸殑鏁版嵁 涓嶅厑璁稿湪杩欎慨鏀瑰弬鏁?
        const int16 mt9v03x_set_confing_buffer[MT9V03X_CONFIG_FINISH][2]=
        {
            {MT9V03X_INIT,              0},                                     // 鎽勫儚澶村紑濮嬪垵濮嬪寲

            {MT9V03X_AUTO_EXP,          MT9V03X_AUTO_EXP_DEF},                  // 鑷姩鏇濆厜璁剧疆
            {MT9V03X_EXP_TIME,          MT9V03X_EXP_TIME_DEF},                  // 鏇濆厜鏃堕棿
            {MT9V03X_FPS,               MT9V03X_FPS_DEF},                       // 鍥惧儚甯х巼
            {MT9V03X_SET_COL,           MT9V03X_W * (perfect_proportion + 1)},  // 鍥惧儚鍒楁暟閲?
            {MT9V03X_SET_ROW,           MT9V03X_H * (perfect_proportion + 1)},  // 鍥惧儚琛屾暟閲?
            {MT9V03X_LR_OFFSET,         MT9V03X_LR_OFFSET_DEF},                 // 鍥惧儚宸﹀彸鍋忕Щ閲?
            {MT9V03X_UD_OFFSET,         MT9V03X_UD_OFFSET_DEF},                 // 鍥惧儚涓婁笅鍋忕Щ閲?
            {MT9V03X_GAIN,              MT9V03X_GAIN_DEF},                      // 鍥惧儚澧炵泭
            {MT9V03X_PCLK_MODE,         MT9V03X_PCLK_MODE_DEF},                 // 鍍忕礌鏃堕挓妯″紡
        };
        return_state = mt9v03x_sccb_set_config(mt9v03x_set_confing_buffer);
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    MT9V03X 鎽勫儚澶村垵濮嬪寲
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           0-鎴愬姛 x-澶辫触
// 浣跨敤绀轰緥     mt9v03x_init();
// 澶囨敞淇℃伅     
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_init (void)
{
    uint8 return_state  = 0;
    
    SCB_DisableICache();
    SCB_DisableDCache(); 
    
    mt9v03x_h_num = MT9V03X_H;
    mt9v03x_w_num = MT9V03X_W;
    
    if(mt9v03x_h_num == 60 && mt9v03x_w_num == 94)      // 瀹岀編缂╁噺姣斾緥 鍙噰闆嗗埌瀹屾暣姣斾緥鍥惧儚
    {
        perfect_proportion = 1;
    }
    
    do
    {
        return_state = mt9v03x_sccb_init();
        
        if(return_state)
        {
            break;
        }
        
        mt9v03x_trig_init();
        
    }while(0);

    SCB_EnableICache();
    SCB_EnableDCache(); 
    
    return return_state;
}


