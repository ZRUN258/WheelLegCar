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
* 2024-01-24       pudding           鏂板SBUS鎺ユ敹鍑芥暟
********************************************************************************************************************/

#include "zf_device_type.h"

static void type_default_callback (void);

wireless_type_enum  wireless_type                   = NO_WIRELESS;
callback_function   wireless_module_uart_handler    = type_default_callback;        // 鏃犵嚎涓插彛鎺ユ敹涓柇鍑芥暟鎸囬拡锛屾牴鎹垵濮嬪寲鏃惰缃殑鍑芥暟杩涜璺宠浆
callback_function   uart_receiver_handler           = type_default_callback;        // SBUS涓插彛鎺ユ敹鏈轰腑鏂嚱鏁版寚閽堬紝鏍规嵁鍒濆鍖栨椂璁剧疆鐨勫嚱鏁拌繘琛岃烦杞?

tof_type_enum       tof_type                        = NO_TOF;
callback_function   tof_module_exti_handler         = type_default_callback;        // ToF 妯″潡 INT 鏇存柊涓柇

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    榛樿鍥炶皟鍑芥暟
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     
// 澶囨敞淇℃伅     淇濇姢鎬у啑浣欒璁?闃叉鍦ㄦ病鏈夊垵濮嬪寲璁惧鐨勬椂鍊欒窇椋?
//-------------------------------------------------------------------------------------------------------------------
static void type_default_callback (void)
{

}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    璁剧疆鏃犵嚎妯″潡绫诲瀷
// 鍙傛暟璇存槑     type_set        閫夊畾鐨勬棤绾挎ā鍧楃被鍨?
// 鍙傛暟璇存槑     uart_callback   璁惧鐨勪覆鍙ｅ洖璋冨嚱鏁?
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     set_wireless_type(WIRELESS_UART, uart_callback);
// 澶囨敞淇℃伅     涓€鑸敱鍚勬憚鍍忓ご鍒濆鍖栧唴閮ㄨ皟鐢?
//-------------------------------------------------------------------------------------------------------------------
void set_wireless_type (wireless_type_enum type_set, callback_function wireless_callback)
{
    wireless_type = type_set;
    if (RECEIVER_UART == wireless_type)
    {
        uart_receiver_handler  = ((wireless_callback == NULL) ? (type_default_callback) : (wireless_callback));
    }
    else
    {
        wireless_module_uart_handler  = ((wireless_callback == NULL) ? (type_default_callback) : (wireless_callback));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    璁剧疆 ToF 妯″潡绫诲瀷
// 鍙傛暟璇存槑     type_set        閫夊畾鐨?ToF 妯″潡绫诲瀷
// 鍙傛暟璇存槑     exti_callback   璁惧鐨勫閮ㄤ腑鏂洖璋冨嚱鏁?
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     set_tof_type(TOF_DL1A, dl1a_int_handler);
// 澶囨敞淇℃伅     涓€鑸敱鍚勬憚鍍忓ご鍒濆鍖栧唴閮ㄨ皟鐢?
//-------------------------------------------------------------------------------------------------------------------
void set_tof_type (tof_type_enum type_set, callback_function exti_callback)
{
    tof_type = type_set;
    tof_module_exti_handler = ((exti_callback == NULL) ? (type_default_callback) : (exti_callback));
}


