/*
 * pid.c
 *
 *  Created on: 2025年3月2日
 *      Author: xy314
 */


#include "pid.h"


float param_pid[9] = {0,0,0,0,0,0,0,0,0};

/*角速度环的pid参数*/
PID angular_speed_cycle={
        .kp =   1.35f,//1.35f
        .ki =   0.0f,
        .kd =   0.30f,//0.30f
        .integral_limit =   0,
        .output_limit   =   10000.0f,
        .output_offset  =   0,
        .low_pass = 1.0f,
};

/*角度环的pid参数*/
PID angle_cycle={
        .kp =   280.0f,//280
        .ki =   0.000f,
        .kd =   80.00f,//125
        .integral_limit =   2000.0f,
        .output_limit   =   10000.0f,
        .output_offset  =   0,
        .low_pass = 1.0f,
};


/*速度环的pid参数*/
PID speed_cycle={
        .kp =   8.0f,//5
        .ki =   0.03f,//0.03
        .kd =   100.0f,//10
        .integral_limit =   20.0f,
        .output_limit   =   30.0f,
        .low_pass = 0.1f,
};

/*速度环的pid参数*/
PID speed_steer_cycle={
        .kp =   8.5f,//10
        .ki =   0.00f,//0.04
        .kd =   5.00f,//5
        .integral_limit =   20.0f,
        .output_limit   =   40.0f,//30
        .output_offset  =    0,
        .low_pass = 1.0f,
};



typedef struct
{
    float finalCenterError[3];                                       //摄像头最终数据（中线偏差）
    float err;
    float last_err;

    float kp;
    float kp_sq;
    float kd;
    float kd_gyro;

    float kp_out;
    float kp_sq_out;
    float kd_out;
    float kd_gyro_out;

    float low_pass;

    float out;

}camERR_t;




//==========转向环================//
camERR_t                    camERR = {
        .kp = 0.2f,
        .kp_sq = 0.00f,
        .kd = 1.3f,
        .kd_gyro = 0.0f,
        .low_pass = 0.5f,
};


PID turn_inv_cycle={
        .kp =   1.4f,
        .ki =   0.0f,
        .kd =   0.0f,
        .integral_limit =   0.0f,
        .output_limit   =   50.0f,
        .output_offset  =   0,
        .low_pass = 1.0,
};



//============转向角速度环=============//
PID speed_yaw_cycle={
        .kp =   500.0f,//420
        .ki =   0.0f,//10
        .kd =   12000.0f,//11250
        .integral_limit =   2000.0f,
        .output_limit   =   10000.0f,
        .output_offset  =   0,
        .low_pass = 1.0,
};




//============差速补偿=============//
PID v_sub_cycle={
        .kp =   650.0f,//650
        .ki =   0.0f,
        .kd =   0.0f,
        .integral_limit =   0.0f,
        .output_limit   =   2000.0f,
        .output_offset  =   0,
        .low_pass = 1.0,
};







//============腿环=============//
PID leg_cycle={
        .kp =   0.07f,//0.07
        .ki =   0.0f,
        .kd =   3.50f,//3.5
        .integral_limit =   10.0f,
        .output_limit   =   10.0f,
        .output_offset  =   0,
        .low_pass = 0.5,
};



//============位置环=============//
//PID pos_cycle={
//        .kp =   4.0f,
//        .ki =   0.00f,
//        .kd =   0.0f,
//        .integral_limit =   5.0f,
//        .output_limit   =   10.0f,
//        .output_offset  =   0,
//        .low_pass = 0,
//};



//PID yaw_cycle={
//        .kp =   14.0f,
//        .ki =   0.3f,
//        .kd =   0.00f,
//        .integral_limit =   4000.0f,
//        .output_limit   =   10000.0f,
//        .output_offset  =   0,
//        .low_pass = 1.0,
//};


//PID vr_cycle={
//        .kp =   2.2f,
//        .kd =   0.00f,
//        .ki =   0.0f,
//        .integral_limit =   4000.0f,
//        .output_limit   =   10000.0f,
//        .output_offset  =   0,
//        .low_pass = 0.1,
//};





float test_pid_controler(float target, float feedback,PID *pid,float threshold_A,float threshold_B)
{
    // 更新误差
    pid->last_err = pid->err;
    pid->err = target - feedback;

    // 计算比例项
    pid->proportion = pid->kp * pid->err;

    // 计算微分项
    pid->differential = pid->kd * (pid->err - pid->last_err);
    pid->differential = pid->low_pass * pid->differential + (1 - pid->low_pass) * pid->differential_last;
    pid->differential_last = pid->differential;


    if(fabs(pid->err)>=threshold_A + threshold_B)
        pid->integral += 0.0f;
    else if(fabs(pid->err)<threshold_A + threshold_B && fabs(pid->err)>threshold_B)
    {
        float alpha = ( threshold_A  + threshold_B - fabs(pid->err) )/ threshold_A ;
        pid->integral += pid->ki * alpha * pid->err;
    }
    else
        pid->integral += pid->ki * pid->err;


    // 计算积分项、积分限幅
    if(pid->integral > pid->integral_limit)   pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit)  pid->integral = -pid->integral_limit;

    // 计算输出量
    pid->output = pid->proportion + pid->integral + pid->differential;
    if(pid->output + pid->output_offset > pid->output_limit)   pid->output = pid->output_limit;
    if(pid->output + pid->output_offset < -pid->output_limit)  pid->output = -pid->output_limit;
    return pid->output;
}



//位置式pid
float pid_controler(float target, float feedback,PID *pid)
{
    // 更新误差
    pid->last_err = pid->err;
    pid->err = target - feedback;

    // 计算比例项
    pid->proportion = pid->kp * pid->err;

    // 计算积分项、积分限幅
    pid->integral += pid->ki * pid->err;
    if(pid->integral > pid->integral_limit)   pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit)  pid->integral = -pid->integral_limit;

    // 计算微分项
    pid->differential = pid->kd * (pid->err - pid->last_err);
    pid->differential = pid->low_pass * pid->differential + (1 - pid->low_pass) * pid->differential_last;
    pid->differential_last = pid->differential;

    // 计算输出量
    pid->output = pid->proportion + pid->integral + pid->differential;
    if(pid->output + pid->output_offset > pid->output_limit)   pid->output = pid->output_limit;
    if(pid->output + pid->output_offset < -pid->output_limit)  pid->output = -pid->output_limit;
    return pid->output;
}

//增量式pid
float pid_controler_incremental(float target, float feedback,PID *pid)
{
    // 更新误差
    pid->last_last_err = pid->last_err;
    pid->last_err = pid->err;
    pid->err = target - feedback;

    // 计算比例项
    pid->proportion = pid->kp * (pid->err - pid->last_err);

    // 计算积分项
    pid->integral = pid->ki * pid->err;

    // 计算微分项
    pid->differential = pid->kd * (pid->err - 2 * pid->last_err + pid->last_last_err);

    pid->output += pid->proportion + pid->integral + pid->differential;
    if(pid->output > pid->output_limit)   pid->output = pid->output_limit;
    if(pid->output < -pid->output_limit)  pid->output = -pid->output_limit;

    return pid->output;

}


const int  num_area = 8;
float e_membership_values[7] = {-3,-2,-1,0,1,2,3}; //输入e的隶属值
float ec_membership_values[7] = { -3,-2,-1,0,1,2,3 };//输入de/dt的隶属值
float kp_menbership_values[7] = { -3,-2,-1,0,1,2,3 };//输出增量kp的隶属值
float ki_menbership_values[7] = { -3,-2,-1,0,1,2,3 }; //输出增量ki的隶属值
float kd_menbership_values[7] = { -3,-2,-1,0,1,2,3 };  //输出增量kd的隶属值

int  Kp_rule_list[7][7] = { { 3,  2,  1,  0,  1,  2,  3},     //kp规则表
                            { 1,  0, -1, -2, -1,  0,  1},
                            {-1, -2, -3, -3, -3, -2, -1},
                            {-3, -3, -3, -3, -3, -3, -3},
                            {-1, -2, -3, -3, -3, -2, -1},
                            { 1,  0, -1, -2, -1,  0,  1},
                            { 3,  2,  1,  0,  1,  2,  3} };

int  Ki_rule_list[7][7] = { { -3,  -1,  1,  3,  1,  -1, -3},     //kp规则表
                            { -3,  -2,  0,  2,  0,  -2, -3},
                            { -3,  -3, -1,  1, -1,  -3, -3},
                            { -3,  -3, -2,  0, -2,  -3, -3},
                            { -3,  -3, -2, -1, -2,  -3, -3},
                            { -3,  -3, -3, -2, -3,  -3, -3},
                            { -3,  -3, -3, -3, -3,  -3, -3}, };

int  Kd_rule_list[7][7] = { { 3,  2,  1,  0,  1,  2,  3},     //kp规则表
                            { 2,  1,  0, -1,  0,  1,  2},
                            { 1,  0, -1, -2, -1,  0,  1},
                            { 0, -1, -2, -3, -2, -1,  0},
                            { 1,  0, -1, -2, -1,  0,  1},
                            { 2,  1,  0, -1,  0,  1,  2},
                            { 3,  2,  1,  0,  1,  2,  3} };

/************************模糊PID控制***********************/
//相关变量的初始化
PID  fpid_speed;
PID  fpid_angle;
void FuzzyPID_init(void)
{
//角度模糊pid
    fpid_angle.errsum = 0;
    fpid_angle.last_err = 0;
    fpid_angle.e_max = 41.5;
    fpid_angle.e_min = -12.6;
    fpid_angle.ec_max = 4;
    fpid_angle.ec_min = -3;
    fpid_angle.kp_max = 212.1;
    fpid_angle.kp_min = 150;
    fpid_angle.ki_max = 0;
    fpid_angle.ki_min = 0;
    fpid_angle.kd_max = 282.8;
    fpid_angle.kd_min = 200;
    fpid_angle.output_limit=10000;
//速度模糊pid
    fpid_speed.errsum = 0;
    fpid_speed.last_err = 0;
    fpid_speed.e_max = 1.9;//1;
    fpid_speed.e_min = -1;//-0.65;
    fpid_speed.ec_max = 1.5;//0.25;
    fpid_speed.ec_min = -1.5;//-0.25;
    fpid_speed.kp_max = 33.936;//28.28;
    fpid_speed.kp_min = 24;//20;
    fpid_speed.ki_max = 0.10808;//0.08484;
    fpid_speed.ki_min = 0.072;//0.06;
    fpid_speed.kd_max = 254.52;// 212.1;
    fpid_speed.kd_min = 180;//150;
    fpid_speed.output_limit=40;
    fpid_speed.output_offset=5;

    fpid_speed.kp = 0;
    fpid_speed.ki = 0;
    fpid_speed.kd = 0;

    fpid_speed.qdetail_kp = 0;
    fpid_speed.qdetail_ki = 0;
    fpid_speed.qdetail_kd = 0;

}

//区间映射函数,
float FuzzyPID_Quantization(float maximum,float minimum,float x)
{
    float qvalues= 6.0 *(x-minimum)/(maximum - minimum)-3;
    return qvalues;
}

//反区间映射函数
float FuzzyPID_Inverse_quantization(float maximum, float minimum, float qvalues)
{
    float x = (maximum - minimum) *(qvalues + 3)/6 + minimum;
    return x;
}


//模糊PID控制实现函数/
float FuzzyPID_controller(float target,float feedback,PID *pid)
{

    pid->detail_kp = 0;
    pid->detail_ki = 0;
    pid->detail_kd = 0;

    pid->qdetail_kp = 0;
    pid->qdetail_ki = 0;
    pid->qdetail_kd = 0;

    // 更新误差
    pid->last_err = pid->err;
    pid->err = target - feedback;

    //计算de/dt
    pid->err_c = pid->err - pid->last_err;


    pid->errsum += pid->err;
    if(pid->errsum>-30) pid->errsum=-30;
    pid->qerr = FuzzyPID_Quantization(pid->e_max, pid->e_min, pid->err);
    pid->qerr_c = FuzzyPID_Quantization(pid->ec_max, pid->ec_min, pid->err_c);

    //输入err与derr/dt,隶属度计算函数

        //err的隶属度
        if (pid->qerr > e_membership_values[0] && pid->qerr < e_membership_values[6])//在论域6段之内
           {
               for (int i = 0; i < num_area - 2; i++)//生成e隶属度在规则表的索引
               {
                   if (pid->qerr >= e_membership_values[i] && pid->qerr <= e_membership_values[i + 1])
                   {
                       pid->e_gradmembership[0] = - (pid->qerr - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
                       pid->e_gradmembership[1] = 1 + (pid->qerr - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
                       pid->e_grad_index[0] = i;
                       pid->e_grad_index[1] = i + 1;
                       break;
                   }
               }
           }
           else
           {
               if (pid->qerr <= e_membership_values[0])//在论域6段左侧
               {
                   pid->e_gradmembership[0] = 1;
                   pid->e_gradmembership[1] = 0;
                   pid->e_grad_index[0] = 0;
                   pid->e_grad_index[1] = -1;
               }
               else if (pid->qerr >= e_membership_values[6])//在论域6段右侧
               {
                   pid->e_gradmembership[0] = 1;
                   pid->e_gradmembership[1] = 0;
                   pid->e_grad_index[0] = 6;
                   pid->e_grad_index[1] = -1;
               }
           }

        //de/dt的隶属度
        if (pid->qerr_c > ec_membership_values[0] && pid->qerr_c < ec_membership_values[6])
           {
               for (int i = 0; i < num_area - 2; i++)
               {
                   if (pid->qerr_c >= ec_membership_values[i] && pid->qerr_c <= ec_membership_values[i + 1])
                   {
                       pid->ec_gradmembership[0] = -(pid->qerr_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
                       pid->ec_gradmembership[1] = 1 + (pid->qerr_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
                       pid->ec_grad_index[0] = i;
                       pid->ec_grad_index[1] = i + 1;
                       break;
                   }
               }
           }
           else
           {
               if (pid->qerr_c <= ec_membership_values[0])
               {
                   pid->ec_gradmembership[0] = 1;
                   pid->ec_gradmembership[1] = 0;
                   pid->ec_grad_index[0] = 0;
                   pid->ec_grad_index[1] = -1;
               }
               else if (pid->qerr_c >= ec_membership_values[6])
               {
                   pid->ec_gradmembership[0] = 1;
                   pid->ec_gradmembership[1] = 0;
                   pid->ec_grad_index[0] = 6;
                   pid->ec_grad_index[1] = -1;
               }
           }

    //获取输出增量kp,ki,kd的总隶属度

        for (int i = 0; i <= num_area - 1; i++)
              {
                fpid_speed.KpgradSums[i] = 0;
                fpid_speed.KigradSums[i] = 0;
                fpid_speed.KdgradSums[i] = 0;
              }

        for (int i=0;i<2;i++)
          {
              if (pid->e_grad_index[i] == -1)
              {
               continue;
              }
              for (int j = 0; j < 2; j++)
              {
                  if (pid->ec_grad_index[j] != -1)
                  {
                      int indexKp = Kp_rule_list[pid->e_grad_index[i]][pid->ec_grad_index[j]] + 3;
                      int indexKi = Ki_rule_list[pid->e_grad_index[i]][pid->ec_grad_index[j]] + 3;
                      int indexKd = Kd_rule_list[pid->e_grad_index[i]][pid->ec_grad_index[j]] + 3;
                      pid->KpgradSums[indexKp] = pid->KpgradSums[indexKp] + (pid->e_gradmembership[i] * pid->ec_gradmembership[j]);
                      pid->KigradSums[indexKi] = pid->KigradSums[indexKi] + (pid->e_gradmembership[i] * pid->ec_gradmembership[j]);
                      pid->KdgradSums[indexKd] = pid->KdgradSums[indexKd] + (pid->e_gradmembership[i] * pid->ec_gradmembership[j]);
                  }
                  else
                  {
                    continue;
                  }

              }
          }

    //计算输出增量kp,kd,ki对应论域值

        for (int i = 0; i < num_area - 1; i++)
        {
            pid->qdetail_kp += kp_menbership_values[i] * pid->KpgradSums[i];
            pid->qdetail_ki += ki_menbership_values[i] * pid->KigradSums[i];
            pid->qdetail_kd += kd_menbership_values[i] * pid->KdgradSums[i];
        }



    pid->detail_kp = FuzzyPID_Inverse_quantization(pid->kp_max, pid->kp_min, pid->qdetail_kp);
    pid->detail_ki = FuzzyPID_Inverse_quantization(pid->ki_max, pid->ki_min, pid->qdetail_ki);
    pid->detail_kd = FuzzyPID_Inverse_quantization(pid->kd_max, pid->kd_min, pid->qdetail_kd);

    pid->kp = pid->detail_kp;
    pid->ki = pid->detail_ki;
    pid->kd = pid->detail_kd;

    pid->output = pid->kp*pid->err  +pid->ki * pid->errsum + pid->kd * (pid->err - pid->last_err);
    if(pid->output+ 10> pid->output_limit)   pid->output = pid->output_limit;
    if(pid->output+10 < -pid->output_limit)  pid->output = -pid->output_limit;

    return pid->output;
}












