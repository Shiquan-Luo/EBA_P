#ifndef _POSPID_H_
#define _POSPID_H_

#include <iostream>

struct pos_pid {
	// def
	double err;//����ƫ��ֵ
	double err_last;//������һ��ƫ��ֵ
	double Kp, Ki, Kd;//������������֡�΢��ϵ��
	double integral;//�������ֵ
	double output;//  �������

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