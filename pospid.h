#ifndef _POSPID_H_
#define _POSPID_H_

#include <iostream>

struct pos_pid {
	// def
	double err;//定义偏差值
	double err_last;//定义上一个偏差值
	double Kp, Ki, Kd;//定义比例、积分、微分系数
	double integral;//定义积分值
	double output;//  定义输出

	// constructor
	pos_pid(double kp, double ki, double kd);

	// initialise
	int PID_init(double kp = 0.02, double ki = 0.015, double kd = 0.2);

	// adjust pid parameters
	void PID_adjust(double kp , double ki, double kd);

	// realize
	double PID_cal(double error);

	double Saturation(double x, double t);
};



#endif