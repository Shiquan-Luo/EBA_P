#include <iostream>
#include <iomanip> 
#include <cmath>
#include <fstream>
#include <string>
#include <ctime>
#include <vector>
#include "slidingmode.h"
#include "pospid.h"
#include "LowPassFilter.hpp"
#include "td.h"

using namespace std;

double pi = acos(-1);

TD omega_td(350,1e-5);

//slidingmode SM_controllor(1, 0.3);
slidingmode SM_controllor(35, 25);

// ESO parameter
double w0 = 48;
double ESO_K1 = 3*w0;
double ESO_K2 = 3*w0*w0;

pos_pid Theta_pid(47, 0, 0);
pos_pid Omega_pid(5, 0.0001, 0);
pos_pid Current_pid(8, 0.005, 0);

pos_pid Velocity_pid(5.5, 1e-9, 0.05);
pos_pid Accleration_pid(0.5, 0, 0);
double t_init=0.3;
double t;
double t_last=0;
// desired press and theta0
double velocity_d = 0;
double Sliprate_d = 0.05;
double T_press_d;
double Press_d;
double theta_d;
double omega_d;
double current_d;
double acc_d = -3.048;

// sliding mode control based on sliprate, pressure, and angular speed of the wheeel
double miu_max = 0.117; 
double best_sliprate = 0.117;

// Parameters

double x[21] = { 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0.02,
                 120*0.514, 0, 0, 0, 0,
                 0 ,15, 120*0.514/0.580,0,120 * 0.514 / 0.580 ,
                 0};

double xMid[21] = { 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0.02,
                    120 * 0.514, 0, 0, 0, 0,
                    0 ,15, 120*0.514/0.580,0,120 * 0.514 / 0.580 ,
                    0};

double xNext[21] = { 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0.02,
                     120 * 0.514, 0, 0, 0, 0,
                     0 ,15,120 * 0.514 / 0.580,0,120 * 0.514 / 0.580 ,
                    0 };

double dx[21] = { 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0,
                  0};


// EBA部分
// x0 E1_i 电机电流
// x1 E1_dtheta0
// x2 E2_theta0
// x3 E2_dtheta1
// x4 E2_theta1
// x5 E2_dtheta2
// x6 E2_theta2
// x7 E2_dtheta3
// x8 E2_theta3
// x9 E3_x
// x10 Vx
// x11 x
// x12 Vy
// x13 y
// x14 omega1
// x15 omega2，
// x16 temperature of brake disc
// x17 omega2_ref
// x18 domega2_ref
// x19 omega_observed
// x20 disturbance beta


// 电机参数

// 电机部分 E1
double E1_Ki = 1.6;
double E1_Ce = 0.9;
double E1_L =  1.2*1E-3;
double E1_R = 16;
double E1_Km = 1.028;
double E1_Jm = 1.5E-4;
double E1_bm = 0.2 * E1_Jm;


// --------
double E1_Umax = 270;
double E1_percent = 0.1;
double E1_U = E1_Umax*E1_percent;
double E1_P;
double E1_T0;
double E1_Tm;

double E1_KK = E1_Umax / E1_L; // 归一化acc

// 齿轮组部分 E2
double E2_Rbs = 0.012;
double E2_Jbs = 4E-4;
double E2_Bbs = 0.2 * E2_Jbs;
double E2_Rsmall = 0.01;
double E2_Jsmall = 5E-6;
double E2_Bsmall = 0.2 * E2_Bsmall;
double E2_Rbig = 0.105;
double E2_Jbig = 5.5E-4;
double E2_Bbig = 0.2 * E2_Jbig;
double E2_eta = 0.9;
double E2_Kg = 1E8;
double E2_Kz1 = 1*1E7;
double E2_Kz2 = 1*1E7;
double E2_ib = E2_Rbig / E2_Rsmall;
// --------
double E2_Ms;
double E2_T1;
double E2_T2;
double E2_T3;
double E2_F;


// 丝杆部分 E3
double E3_lambda = pi / 12;
double E3_K = 5E6;
double E3_x0 = 0.02;
// --------
double E3_Press;
double E3_B = 0.2; //粘滞阻尼系数

// 刹车盘部分 E4
double E4_K_init = 0.25283;
double E4_K = 0.25283;
double E4_A = 8;
// --------
double E4_S;
double E4_M;
//--------- 刹车盘热力学
double E4_q1;
double E4_q2;
double E4_R1 = 0.281;
double E4_R2 = 0.454;
double E4_Pr = 0.7;
// double E4_lambda = 6.14e-6;
double E4_lambda = 0.023;
double E4_l = 0.00215;
double E4_miu = 1.48e-5;
double E4_T0 = 15;
double E4_C;
double E4_Area = 3.1415 * (pow(E4_R2, 2) - pow(E4_R1, 2));
double E4_m = E4_Area * E4_l * 1.75e3;


// 飞机部分 A1
double A1_M = 60000;
double A1_g = 9.8;
double A1_I = 2810000;
double A1_a2 = 1.4;
double A1_a1 = 11;
double A1_hc = 4.4;
double A1_hs = 0;
double A1_ht = 0;
double A1_Rh2 = 0.580;
double A1_Rh1 = 0.400;
double A1_rho = 1.225;
double A1_CD = 0.2;
double A1_Area = 120;
double A1_CY = 1.2;
double A1_CY_init = 1.4;
// --------
double A1_T0;
double A1_Ff1;
double A1_Ff2;
double A1_Y;
double A1_Q;
double A1_N1;
double A1_N2;

// 起落架部分 A2
double A2_L0 = 0;
double A2_C = 50000;

// --------
double A2_N1;
double A2_N2;
double A2_N;

// 前轮胎部分 A3
double A3_J = 2.4 * 2;
double A3_D = 0.8;
double A3_C = 1.5344;
double A3_B = 14.0326;
// --------
double A3_mu;
double A3_Mf;
double A3_sliprate = 1;
double A3_V;
double A3_mid;

// 主轮胎部分 A4
double A4_J = 34;
double A4_D = 0.8;
double A4_C = 1.5344;
double A4_B = 14.0326;
// --------
double A4_mu;
double A4_Mf;
double A4_sliprate = 1;
double A4_V;
double A4_mid;

// Controller parameters

double X1,F1, G1;
double Miu_d,X1d,DX1d;
double Ueq;

double N2_est,N1_est;
double N2_err = 1;
double SNR = 0.1;


// SSMO part
double y_est ,N_est ,Fy_est;
double dy_est, dN_Est, dFy_est;

// TD paramter
double* TD_output;

double In_transition=true;

// 符号函数
int eso_sgn(double x) {
    if (x > 0) {
        return 1;
    }
    if (x < 0) {
        return -1;
    }
    return 0;
}

// 插值函数
double LLinear(double x)
{
    double y;

    y = (62000 * pow(x, 5) - 54000 * pow(x, 4) + 17000 * pow(x, 3) - 2200 * pow(x, 2) + 1200 * x + 6.2) * pow(10, 4);

    return y;
}


// 计算控制器输出 
void Control_calculation(double t) {

    double Kn2 = 2 * (A1_a2 + A4_mu * A1_hc) / (A1_a1 - 0.0012* A1_hc);
    N2_est = (A1_M * A1_g - (A1_rho * x[10] * x[10] * A1_CY_init * A1_Area / 2)) / (Kn2 + 2);
    N1_est = N2_est * Kn2;
    /*
        velocity_d = 120 * 0.514 - 3.048 * (t - 0.3);
    acc_d = 3.048 + Velocity_pid.PID_cal(x[10] - velocity_d);
    Miu_d = 0.5 / N2_est * (A1_T0 - A1_Q - N1_est * A3_mu - A1_M * acc_d);
    Sliprate_d = 0.04 +0.06* (1 - exp(-t+0.3));
    //Sliprate_d = 1 / A4_B * tan(1 / A4_C * asin(Miu_d / A4_D));
    */

    X1d = x[10] * (1 - Sliprate_d)/A1_Rh2;
    // sliding mode
    //X1 = A4_sliprate;
    //F1 = (1 - A4_sliprate) * dx[10] / x[10] - (A1_Rh2 * A1_Rh2 / x[10] / A4_J) * A4_mu * N2_est;
    //G1 = E4_K * A1_Rh2 * 8 / (x[10] * A4_J);
    X1 = x[15];
    F1 = A4_mu * N2_est * A1_Rh2 / A4_J;
    G1 = -1 / A4_J;
    DX1d = x[18];
    /*
      int temp0 = SM_controllor.update(X1, DX1d, G1, F1);
    int temp1 = SM_controllor.forefeed(X1d);
    double sm_cal = SM_controllor.cal();
    T_press_d = sm_cal - 1 / G1 * x[20];
    Press_d = T_press_d / (E4_A * E4_K_init);
    theta_d = Press_d / E3_K / tan(E3_lambda) / E2_Rbs * E2_ib;
    omega_d = Theta_pid.PID_cal(theta_d - x[2]);
    current_d = Omega_pid.PID_cal(omega_d - x[1]);
    E1_U = Current_pid.PID_cal(current_d - x[0] * E1_KK);  
    */


}

double Saturat(double x, double t) {
    if (x > abs(t)) {
        return  abs(t);
    }
    if (x < -abs(t)) {
        return  -abs(t);
    }
    return x;
}


//恢复默认步长
double ResetStepSize(double dt_default)
{
    return dt_default;
}




// 修改至此↓
// 成员函数定义
int function(double t)
{
    // 系统状态方程

    // 电机部分
    dx[0] = (E1_U - (E1_Ki + E1_R) * xMid[0] * E1_KK - E1_Ce * xMid[1]) / E1_Umax;
    dx[1] = (E1_Tm + E1_T0 - E1_bm * xMid[1]) / E1_Jm;
    
    // 齿轮组部分    
    dx[2] = xMid[1];
    dx[3] = (E2_T1 - E2_F * E2_Rsmall - E2_Bsmall * xMid[3]) / E2_Jsmall;
    dx[4] = xMid[3];
    dx[5] = (E2_T2 + E2_F * E2_Rbig - E2_Bbig * xMid[5]) / E2_Jbig;
    dx[6] = xMid[5];
    dx[7] = (E2_T3 - E2_Ms - E2_Bbs * xMid[7]) / E2_Jbs;   // 

 
    dx[8] = xMid[7];

    // 丝杆部分
    dx[9] = E2_Rbs * xMid[7] * tan(E3_lambda);   //dx



    // 飞机部分
    dx[10] = (A1_T0 - A1_Q - A1_Ff1 - 2 * A1_Ff2) / A1_M;
    dx[11] = xMid[10];
    dx[12] = (A1_Y + 2 * A1_N2 + A1_N1 - A1_M * A1_g) / A1_M - 0.2 * xMid[12];

    // 起落架部分
    dx[13] = -xMid[12];

    // 前轮胎部分
    if (A3_sliprate < 0)
    {
        dx[14] = -A3_Mf / A3_J;
    }
    else
    {
        dx[14] = A3_Mf / A3_J;
    }

    // 后轮胎部分
    if (A4_sliprate < 0)
    {
        dx[15] = -(A4_Mf + E4_M) / A4_J;
    }
    else if(A4_sliprate > 1)
    {
        dx[15] = (A4_Mf + E4_M) / A4_J;
    }
    else
    {
        dx[15] = (A4_Mf - E4_M) / A4_J;
    }
    // 刹车盘温度
    if (x[16] > E4_T0) {
        dx[16] = (E4_q1 - E4_q2) * E4_Area / E4_C / E4_m;
    }
    else {
        dx[16] = (E4_q1 + E4_q1) * E4_Area / E4_C / E4_m;
    }

    // TD
    dx[17] = TD_output[0];
    dx[18] = TD_output[1];
    dx[19] = 1 / A4_J * A4_mu * N2_est*A1_Rh2 - 1 / A4_J * Press_d * E4_A * E4_K_init + xMid[20] + ESO_K1*eso_sgn(xMid[15]-xMid[19]);
    dx[20] = ESO_K2 * eso_sgn(xMid[15] - xMid[19]);

    return 0;
}


int calculate(double t,double h)
{   


    E1_Tm = E1_Km * xMid[0] * E1_KK;
    E1_P = E1_U * xMid[0] * E1_KK;

    E2_Ms = E3_Press * tan(E3_lambda) * E2_Rbs / E2_eta;
    E2_F = E2_Kg * (xMid[4] * E2_Rsmall - xMid[6] * E2_Rbig);
    E2_T1 = E2_Kz1 * (xMid[2] - xMid[4]);
    E1_T0 = -E2_T1;
    E2_T3 = E2_Kz2 * (xMid[6] - xMid[8]);

    E2_T2 = -E2_T3;

    if (xMid[9] > E3_x0)
    {
        E3_Press = E3_K * (xMid[9] - E3_x0);
    }
    else
    {
        E3_Press = 0;
    }

    // 计算刹车盘温度以及结合系数
    double r = (E4_R1 + E4_R2) * 0.5;
    E4_q1 = E4_K * E3_Press * xMid[15] * r / E4_Area;
    E4_q2 = 0.337 * pow(E4_Pr, 0.5) * E4_lambda / pow(E4_miu, 0.8) * pow((r * xMid[15]), 0.8) / pow(E4_l, 0.2) * (xMid[16] - E4_T0)/E4_A;
    E4_C = (- 1.78e-6 * pow(xMid[16], 2) + 0.0029 * xMid[16] + 0.68) * 1e3;
    if (xMid[16] > 800) {
        E4_C = 1.8*1e3;
    }
    if (xMid[16] < 197) {
        E4_K = -6.25e-6 * pow(xMid[16], 2) + 2.5e-3 * xMid[16] + 0.15;
    }
    else {
        E4_K = -1.5e-7 * pow(xMid[16], 2) + 6e-5 * xMid[16] + 0.394;
    }

    E4_S = E3_Press * E4_A;
    E4_M = E4_K * E4_S;

    A2_N1 = LLinear(xMid[13]);
    if (xMid[12] < 0)
    {
        A2_N2 = -xMid[12] * A2_C;
    }
    else
    {
        A2_N2 = xMid[12] * A2_C;
    }

    A2_N = A2_N1 + A2_N2;
    A1_N2 = A2_N;

    A3_Mf = A3_mu * A1_N1 * A1_Rh1;
    A3_V = xMid[14] * A1_Rh1;
    A3_sliprate = (x[10] - A3_V) / x[10];
    A3_mid = A3_C * atan(A3_B * abs(A3_sliprate));
    A3_mu = A3_D * sin(A3_mid);

    A4_Mf = A4_mu * A1_N2 * A1_Rh2;
    A4_V = xMid[15] * A1_Rh2;
    A4_sliprate = (x[10] - A4_V) / x[10];

    A4_mid = A4_C * atan(A4_B * abs(A4_sliprate));
    A4_mu = A4_D * sin(A4_mid);

    A1_T0 = 0;
    A1_Y = A1_rho * x[10] * x[10] * A1_CY * A1_Area / 2;
    A1_Q = A1_rho * x[10] * x[10] * A1_CD * A1_Area / 2;

    A1_Ff1 = A3_mu * A1_N1;
    A1_Ff2 = A4_mu * A1_N2;

    A1_N1 = (2 * A1_N2 * A1_a2 + A1_T0 * A1_ht + 2 * A1_Ff2 * A1_hc) / (A1_a1 - A3_mu * A1_hc);

    TD_output = omega_td.TD_cal(x[10] *(1- Sliprate_d)/A1_Rh2,x[17],x[18], h);
    return 0;
}

// 修改至此↑
double simulation(double h, double t)
{   
    int iteration_num = 0;
    double tol;
    tol = pow(double(10), -6);

    double X0A, X1A, X2A, X3A, X4A, X5A, X6A, X7A, X8A, X9A, X10A, X11A, X12A, X13A, X14A, X15A, X16A, X17A, X18A, X19A, X20A;
    double D=0;

    t_last = t;
    if (t - t_last < 1e-2) {
        //std::cout << "t     " <<t<< std::endl;
    }
    

    xNext[0] = x[0];
    xNext[1] = x[1];
    xNext[2] = x[2];
    xNext[3] = x[3];
    xNext[4] = x[4];
    xNext[5] = x[5];
    xNext[6] = x[6];
    xNext[7] = x[7];
    xNext[8] = x[8];
    xNext[9] = x[9];
    xNext[10] = x[10];
    xNext[11] = x[11];
    xNext[12] = x[12];
    xNext[13] = x[13];
    xNext[14] = x[14];
    xNext[15] = x[15];
    xNext[16] = x[16];
    xNext[17] = x[17];
    xNext[18] = x[18];
    xNext[19] = x[19];
    xNext[20] = x[20];
   
    if (t > t_init) 
    {
        t_last = t;
        double Kn2 = 2 * (A1_a2 + A4_mu * A1_hc) / (A1_a1 - 0.0012 * A1_hc);
        N2_est = (A1_M * A1_g - (A1_rho * x[10] * x[10] * A1_CY_init * A1_Area / 2)) / (Kn2 + 2);
        N1_est = N2_est * Kn2;
        velocity_d = 120 * 0.514 - 3.048 * t;
        acc_d = -3.048 - Velocity_pid.PID_cal(x[10] - velocity_d);
        double acc_d_2 = acc_d - Accleration_pid.PID_cal(dx[10] - acc_d);
        Miu_d = 0.5 / N2_est * (A1_T0 - A1_Q - N1_est * A3_mu - A1_M * acc_d_2);
        if (Miu_d > 1) {
            Miu_d = 1;
        }
        if (Miu_d < 0) {
            Miu_d = 0;
        }
        if (Miu_d > A4_D) {
            Miu_d = A4_D;
        }
        
        Sliprate_d = 1 / A4_B * tan(1 / A4_C * asin(Miu_d / A4_D));
        double pre_sliprate_d = 0.05 + (0.117 - 0.05) * (1 - exp(-1 * (t - t_init)));
        if (pre_sliprate_d <= Sliprate_d) {
            Sliprate_d = pre_sliprate_d;
        }
        else
        {
           
            //In_transition = false;
        }
        
        
        
        /*
        std::cout << Miu_d << std::endl;
        std::cout << A4_D << std::endl;
        std::cout << asin(Miu_d / A4_D) << std::endl;
        std::cout << tan(1 / A4_C * asin(Miu_d / A4_D)) << std::endl;
        std::cout << Sliprate_d << std::endl;
        */

 
        X1d = x[10] * (1 - Sliprate_d) / A1_Rh2;
        X1 = x[15];
        F1 = A4_mu * N2_est * A1_Rh2 / A4_J;
        G1 = -1 / A4_J;
        DX1d = x[18];
        int temp0 = SM_controllor.update(X1, DX1d, G1, F1);
        int temp1 = SM_controllor.forefeed(X1d);
        double sm_cal = SM_controllor.cal();
        T_press_d = sm_cal - 1 / G1 * x[20];
        Press_d = T_press_d / (E4_A * E4_K_init);
        theta_d = Press_d / E3_K / tan(E3_lambda) / E2_Rbs * E2_ib;
        omega_d = Theta_pid.PID_cal(theta_d - x[2]);
        current_d = Omega_pid.PID_cal(omega_d - x[1]);
        E1_U = Current_pid.PID_cal(current_d - x[0] * E1_KK);

        if (t > 0.392) {
           // std::cout<<E1_U<<"\t"<<
        }

        if (E1_U > 270) {
            E1_U = 270;
        }
        if (E1_U < 0) {
            E1_U = 0;
         }
           

    }
  

    while ( 1 )
    {   
        iteration_num ++;
        xMid[0] = (x[0] + xNext[0]) / 2;
        xMid[1] = (x[1] + xNext[1]) / 2;
        xMid[2] = (x[2] + xNext[2]) / 2;
        xMid[3] = (x[3] + xNext[3]) / 2;
        xMid[4] = (x[4] + xNext[4]) / 2;
        xMid[5] = (x[5] + xNext[5]) / 2;
        xMid[6] = (x[6] + xNext[6]) / 2;
        xMid[7] = (x[7] + xNext[7]) / 2;
        xMid[8] = (x[8] + xNext[8]) / 2;
        xMid[9] = (x[9] + xNext[9]) / 2;
        xMid[10] = (x[10] + xNext[10]) / 2;
        xMid[11] = (x[11] + xNext[11]) / 2;
        xMid[12] = (x[12] + xNext[12]) / 2;
        xMid[13] = (x[13] + xNext[13]) / 2;
        xMid[14] = (x[14] + xNext[14]) / 2;
        xMid[15] = (x[15] + xNext[15]) / 2;
        xMid[16] = (x[16] + xNext[16]) / 2;
        xMid[17] = (x[17] + xNext[17]) / 2;
        xMid[18] = (x[18] + xNext[18]) / 2;
        xMid[19] = (x[19] + xNext[19]) / 2;
        xMid[20] = (x[20] + xNext[20]) / 2;

        calculate(t,h);
        function(t);

        X0A = x[0] + h * dx[0];
        X1A = x[1] + h * dx[1];
        X2A = x[2] + h * dx[2];
        X3A = x[3] + h * dx[3];
        X4A = x[4] + h * dx[4];
        X5A = x[5] + h * dx[5];
        X6A = x[6] + h * dx[6];
        X7A = x[7] + h * dx[7];
        X8A = x[8] + h * dx[8];
        X9A = x[9] + h * dx[9];
        X10A = x[10] + h * dx[10];
        X11A = x[11] + h * dx[11];
        X12A = x[12] + h * dx[12];
        X13A = x[13] + h * dx[13];
        X14A = x[14] + h * dx[14];
        X15A = x[15] + h * dx[15];
        X16A = x[16] + h * dx[16];
        X17A = x[17] + h * dx[17];
        X18A = x[18] + h * dx[18];
        X19A = x[19] + h * dx[19];
        X20A = x[20] + h * dx[20];
       D = sqrt(pow(xNext[0] - X0A, 2) + pow(xNext[1] - X1A, 2) + pow(xNext[2] - X2A, 2) + pow(xNext[3] - X3A, 2) + pow(xNext[4] - X4A, 2) + pow(xNext[5] - X5A, 2) + pow(xNext[6] - X6A, 2) + pow(xNext[7] - X7A, 2) + pow(xNext[8] - X8A, 2) + pow(xNext[9] - X9A, 2) + pow(xNext[10] - X10A, 2) 
            + pow(xNext[11] - X11A, 2) + pow(xNext[12] - X12A, 2) + pow(xNext[13] - X13A, 2) + pow(xNext[14] - X14A, 2) + pow(xNext[15] - X15A, 2)+ pow(xNext[16] - X16A, 2) + pow(xNext[17] - X17A, 2) + pow(xNext[18] - X18A, 2) + pow(xNext[19] - X19A, 2)*1e-8 + pow(xNext[20] - X20A, 2)*1e-8);
       D = sqrt(pow(xNext[0] - X0A, 2) + pow(xNext[1] - X1A, 2) + pow(xNext[2] - X2A, 2) + pow(xNext[3] - X3A, 2) + pow(xNext[4] - X4A, 2) + pow(xNext[5] - X5A, 2) + pow(xNext[6] - X6A, 2) + pow(xNext[7] - X7A, 2) + pow(xNext[8] - X8A, 2) + pow(xNext[9] - X9A, 2) + pow(xNext[10] - X10A, 2)
           + pow(xNext[11] - X11A, 2) + pow(xNext[12] - X12A, 2) + pow(xNext[13] - X13A, 2) + pow(xNext[14] - X14A, 2) + pow(xNext[15] - X15A, 2) + pow(xNext[16] - X16A, 2) + pow(xNext[17] - X17A, 2) + pow(xNext[18] - X18A, 2));


        if (D>1)
        {
            h /= 2.0;
            xNext[0] = x[0];
            xNext[1] = x[1];
            xNext[2] = x[2];
            xNext[3] = x[3];
            xNext[4] = x[4];
            xNext[5] = x[5];
            xNext[6] = x[6];
            xNext[7] = x[7];
            xNext[8] = x[8];
            xNext[9] = x[9];
            xNext[10] = x[10];
            xNext[11] = x[11];
            xNext[12] = x[12];
            xNext[13] = x[13];
            xNext[14] = x[14];
            xNext[15] = x[15];
            xNext[16] = x[16];

            if (h == 0)
            {
                cout << "Error: step size become zero!" << endl;
                system("pause");
            }
            continue;
        }

        else if (D < tol)
        {
            break;
        }

        xNext[0] = X0A;
        xNext[1] = X1A;
        xNext[2] = X2A;
        xNext[3] = X3A;
        xNext[4] = X4A;
        xNext[5] = X5A;
        xNext[6] = X6A;
        xNext[7] = X7A;
        xNext[8] = X8A;
        xNext[9] = X9A;
        xNext[10] = X10A;
        xNext[11] = X11A;
        xNext[12] = X12A;
        xNext[13] = X13A;
        xNext[14] = X14A;
        xNext[15] = X15A;
        xNext[16] = X16A;
        xNext[17] = X17A;
        xNext[18] = X18A;
        xNext[19] = X19A;
        xNext[20] = X20A;
    }

    xNext[0] = X0A;
    xNext[1] = X1A;
    xNext[2] = X2A;
    xNext[3] = X3A;
    xNext[4] = X4A;
    xNext[5] = X5A;
    xNext[6] = X6A;
    xNext[7] = X7A;
    xNext[8] = X8A;
    xNext[9] = X9A;
    xNext[10] = X10A;
    xNext[11] = X11A;
    xNext[12] = X12A;
    xNext[13] = X13A;
    xNext[14] = X14A;
    xNext[15] = X15A;
    xNext[16] = X16A;
    xNext[16] = X16A;
    xNext[17] = X17A;
    xNext[18] = X18A;
    xNext[19] = X19A;
    xNext[20] = X20A;

    x[0] = xNext[0];
    x[1] = xNext[1];
    x[2] = xNext[2];
    x[3] = xNext[3];
    x[4] = xNext[4];
    x[5] = xNext[5];
    x[6] = xNext[6];
    x[7] = xNext[7];
    x[8] = xNext[8];
    x[9] = xNext[9];
    x[10] = xNext[10];
    x[11] = xNext[11];
    x[12] = xNext[12];
    x[13] = xNext[13];
    x[14] = xNext[14];
    x[15] = xNext[15];
    x[16] = xNext[16];
    x[17] = xNext[17];
    x[18] = xNext[18];
    x[19] = xNext[19];
    x[20] = xNext[20];

    return h;
}

int main(int argc, char** argv)
{   

    clock_t start = clock();
    double startTime = 0;
    double h_int = 1E-4;
    t = startTime;
    double h = h_int;
    double max_sliprate = 0;
    double t_mark;
    double t_pre = -1;
    double efficiency;
    double max_mu = 0.8;
    ofstream simout("data.txt");
    simout.precision(10);

    while (1)
    {
        efficiency = A4_mu / 0.8;
    
        //CalParam();
        //temp = controllor.update(X1, G1, F1, X2, G2, F2, X3, G3, F3, X4, G4, F4, h);

        h = ResetStepSize(h_int);
        h = simulation(h, t);



        if (A4_sliprate > max_sliprate && t > 1)
        {
            max_sliprate = A4_sliprate;
            t_mark = t;
        }

   
        if (t - t_pre > 0.001)
        {

            if (t > 0.3) {
                
                std::cout << fixed << setprecision(4) << "t:" << "\t" << t << "\t" <<Sliprate_d<< "   Desired  Sliprate:   Actual  " << A4_sliprate << "   Velocity    " << x[10] <<"     Deceleration    " << dx[10] << endl;
                std::cout << fixed << setprecision(4) << "t:" << "\t" << t << "\t" << "E1_U:        " << E1_U        <<"    Mu_1       " <<A3_mu<< endl;
                std::cout << omega_d << "  D Omega  A   " << x[1] << "   " << theta_d<<"    D theta  A   " <<x[2]<< endl;
                std::cout << acc_d << "     D   ACC    A      " <<dx[10]<<"      " <<velocity_d<<"    D     Vel      A     " <<x[10]<< std::endl;
                std::cout << "TD:       desired omega    " << x[10] * (1 - Sliprate_d )/A1_Rh2<<"   TD_ouput    " << x[17] << "   Omega2    " << x[15] << "   Omega2_hat   " << x[19] << endl;
                std::cout << "TD test :       !!!   t   " << t << "\t" << dx[17] << "  dx[17]    dx[18]" << dx[18] << endl;
                std::cout << "ESO test        !!!   t   " << t << "\t" << dx[19] << "  dx[19]    dx[20]" << dx[20] << "   beta     " <<x[20]<< endl;
                std::cout << endl;
            }

            if (t > 0.3) {
                // 0-6  time  velocity  accleration  sliprate  efficiency  Voltage  Support_force 
                simout << t << "\t" << x[10] << "\t" << dx[10] << "\t" << A4_sliprate << "\t" << efficiency << "\t" << E1_U << "\t" << A1_N2 <<"\t";  
                // Temperature  E4_K   y  Desired sliprate  N2_est
                simout <<x[16]<<"\t" <<E4_K<< "\t" <<x[13]<<"\t" << Sliprate_d <<"\t" <<N2_est<<"\t";//7-11
                // Motor angle and torque   Presure Screw_displacement
                simout<<x[2]<<"\t" <<E1_T0 <<"\t" <<E3_Press<<"\t" <<x[9]<< endl;

            }

            t_pre = t;
        }
        
        if (x[10] < 20 * 0.514)
        {
            break;
        }

        t = t + h;


    }
        
    cout << t_mark << " \t " << max_sliprate << endl;
    cout << "congratulations" << endl;
    // do something...
    clock_t end = clock();
    double elapsed_secs = static_cast<double>(end - start) / CLOCKS_PER_SEC;
    std::cout << elapsed_secs << " s" << endl;

	return 0;
}
