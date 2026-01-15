/*********************************************************************************************************************
* CYT4BB Opensourec Library 鍗筹紙 CYT4BB 寮€婧愬簱锛夋槸涓€涓熀浜庡畼鏂?SDK 鎺ュ彛鐨勭涓夋柟寮€婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰绉戞妧
*
* 鏈枃浠舵槸 CYT4BB 寮€婧愬簱鐨勪竴閮ㄥ垎
*
* CYT4BB 寮€婧愬簱 鏄厤璐硅蒋浠?
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
* 鏂囦欢鍚嶇О          zf_device_type
* 鍏徃鍚嶇О          鎴愰兘閫愰绉戞妧鏈夐檺鍏徃
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮€鍙戠幆澧?         IAR 9.40.1
* 閫傜敤骞冲彴          CYT4BB
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
*
* 淇敼璁板綍
* 鏃ユ湡              浣滆€?               澶囨敞
* 2024-01-12       pudding           first version
********************************************************************************************************************/

#ifndef _zf_device_type_h_
#define _zf_device_type_h_

#include "zf_common_typedef.h"

//==============================================瀹氫箟 澶栬 鍙傛暟缁撴瀯浣?=================================================

typedef enum
{
    NO_WIRELESS = 0,                                                            // 鏃犺澶?
    WIRELESS_UART,                                                              // 鏃犵嚎涓插彛
    BLE6A20,                                                                    // 钃濈墮涓插彛
    BLUETOOTH_CH9141,                                                           // 钃濈墮 CH9141
    WIFI_UART,                                                                  // 涓插彛 WiFi
    RECEIVER_UART,                                                              // SBUS閬ユ帶鍣ㄦ帴鏀?
}wireless_type_enum;

typedef enum
{
    NO_TOF = 0,                                                                 // 鏃犺澶?
    TOF_DL1A,                                                                   // DL1A
    TOF_DL1B,                                                                   // DL1B
}tof_type_enum;
//==============================================瀹氫箟 澶栬 鍙傛暟缁撴瀯浣?=================================================


//===========================================澹版槑 鍥炶皟鍑芥暟鎸囬拡鍙婂璁?绫诲瀷==============================================
typedef void (*callback_function)(void);

extern wireless_type_enum   wireless_type;
extern callback_function    wireless_module_uart_handler;                       // 鏃犵嚎涓插彛鎺ユ敹涓柇鍑芥暟鎸囬拡锛屾牴鎹垵濮嬪寲鏃惰缃殑鍑芥暟杩涜璺宠浆
extern callback_function    uart_receiver_handler;                              // SBUS涓插彛鎺ユ敹鏈轰腑鏂嚱鏁版寚閽堬紝鏍规嵁鍒濆鍖栨椂璁剧疆鐨勫嚱鏁拌繘琛岃烦杞?
extern tof_type_enum        tof_type;                                           // ToF 妯″潡 绫诲瀷
extern callback_function    tof_module_exti_handler;                            // ToF 妯″潡 INT 鏇存柊涓柇
//===========================================澹版槑 鍥炶皟鍑芥暟鎸囬拡鍙婂璁?绫诲瀷==============================================


//=============================================澹版槑 涓柇鍥炶皟 鍩虹鍑芥暟================================================
void   set_wireless_type        (wireless_type_enum type_set, callback_function wireless_callback);
void   set_tof_type             (tof_type_enum type_set, callback_function exti_callback);
//=============================================澹版槑 涓柇鍥炶皟 鍩虹鍑芥暟================================================

#endif


