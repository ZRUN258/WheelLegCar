/*
 * balance.h
 *
 *  Created on: 2025年3月13日
 *      Author: mqx
 */
#ifndef BALANCE_H_
#define BALANCE_H_

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#include "pid.h"



#define L_dead_zone_correct           (155)       //左电机正死区
#define L_dead_zone_negative          (-140)      //左电机负死区
#define R_dead_zone_correct           (160)       //右电机正死区
#define R_dead_zone_negative          (-150)      //右电机负死区


#define High_3cm 1
#define High_5cm 0



void                balance_control(void);
void                balance_control_LQR(float V_target, float th);
void                balance_control_PID(void);

void                Turn_Control(void);
float               Turn_Control_inv(void);
float               sliding_window_filter_radius(float new_value);
void                Turn_inv_Control(void);
void                Speed_Control(void);
void                Leg_Control(void);
void                anti_windup(void);


void                dynamic_motor_control(void);
void                dead_compensate(int16 *input_L, int16 *input_R);
float               angle_low_pass(float angle , float alpha);
float               roll_low_pass(float angle , float alpha);
float               gyroz_target_low_pass(float angle , float alpha);
float               sliding_window_filter_gyroz_target(float new_value);
void                single_bridge_param_change(int8 is_single_bridge);
float               yaw_control(void);
float               low_pass(float alpha , float data , float* temp);

extern float anti_windup_speed_B ;
extern float anti_windup_speed_A ;


#endif




