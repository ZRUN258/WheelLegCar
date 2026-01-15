/*
 * pid.h
 *
 *  Created on: 2025年3月2日
 *      Author: xy314
 */
#ifndef PID_H
#define PID_H

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"



typedef struct {
    float kp;                // 比例系数
    float ki;                // 积分系数
    float kd;                // 微分系数
    float err;               // 误差
    float last_err;          // 上次误差
    float last_last_err;     // 上次的上次误差
    float proportion;        // 比例项
    float integral;          // 积分项
    float differential;      // 微分项
    float integral_limit;    // 积分限幅
    float output;            // PID输出
    float output_offset;     //输出偏置
    float output_limit;      // PID输出限幅
    float incremental_data0; //位置式pid用到
    float incremental_data1; //位置式pid用到

    float low_pass;          //D项低通滤波
    float differential_last; //上一次D项

    float errsum;                  //累加和
    float err_c;                    //误差纯微分
    float detail_kp;                //输出增量kp
    float detail_ki;                //输出增量ki
    float detail_kd;                //输出增量kd
    float qerr;                     //区间化的误差
    float qerr_c;                   //区间化的误差微分
    float qdetail_kp;               //增量kp对应论域中的值
    float qdetail_ki;               //增量ki对应论域中的值
    float qdetail_kd;              //增量kd对应论域中的值



   //隶属度
    float gradSums[7];
    float KpgradSums[7];   //输出增量kp总的隶属度
    float KigradSums[7];   //输出增量ki总的隶属度
    float KdgradSums[7];   //输出增量kd总的隶属度
    float e_gradmembership[2];                                              //输入e的隶属度
    float ec_gradmembership[2];                                             //输入de/dt的隶属度
    int e_grad_index[2];                                                    //输入e隶属度在规则表的索引
    int ec_grad_index[2];                                                   //输入de/dt隶属度在规则表的索引


    //参数修改
    float e_max;   //1    //字面意思
    float e_min;   //-0.65
    float ec_max;  //0.25
    float ec_min;  //-0.25
    float kp_max;  //28.26
    float kp_min;  //14.14
    float ki_max;  //0.084
    float ki_min;  //0.042
    float kd_max;  //211.95
    float kd_min;  //106.05

}PID;

extern PID              fpid_angle;
extern PID              fpid_speed;
extern PID              angular_speed_cycle;
extern PID              angle_cycle;
extern PID              speed_cycle;
extern PID              speed_steer_cycle;
extern PID              speed_leg_cycle;
extern PID              pos_cycle;
extern PID              speed_yaw_cycle;
extern PID              v_sub_cycle;
extern PID              turn_inv_cycle;
extern PID              leg_cycle;
extern PID              yaw_cycle;
extern PID              vr_cycle;



extern float param_pid[9];

float test_pid_controler(float target, float feedback,PID *pid,float threshold_A,float threshold_B);
float pid_controler(float target, float feedback,PID *pid);
float pid_controler_incremental(float target, float feedback,PID *pid);


void FuzzyPID_init(void);
float FuzzyPID_controller(float target, float feedback,PID *pid);

float FuzzyPID_Quantization(float maximum,float minimum,float x);
float FuzzyPID_Inverse_quantization(float maximum, float minimum, float qvalues);






#endif


