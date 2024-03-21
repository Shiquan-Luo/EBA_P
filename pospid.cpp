#include "pospid.h"

/*
struct pos_pid{
// def
double err;//定义偏差值
double err_last;//定义上一个偏差值
double Kp,Ki,Kd;//定义比例、积分、微分系数
double integral;//定义积分值
double output;//  定义输出

// initialise
void PID_init(double kp , double ki, double kd);

// adjust pid parameters
// void PID_adjust(double kp , double ki, double kd);

// realize
double PID_cal(double error);
};
*/

pos_pid::pos_pid(double kp, double ki, double kd) {
	err = 0.0;
	err_last = 0.0;
	output = 0.0;
	integral = 0.0;
	Kp = kp;
	Ki = ki;
	Kd = kd;
}

int pos_pid::PID_init(double kp, double ki, double kd){
err=0.0;
err_last=0.0;
output=0.0;
integral=0.0;
Kp=kp;
Ki=ki;
Kd=kd;
return 0;
}

void pos_pid::PID_adjust(double kp, double ki, double kd) {
	Kp = kp;
	Ki = ki;
	Kd = kd;
}

double pos_pid::PID_cal(double error){
err        = error;
integral   += err;
output     = Kp * err + Ki * integral + Kd * (err-err_last);
err_last   = err;
return output;
}


double pos_pid::Saturation(double x, double t) {
	double lim = abs(t);
	if (x > lim) {
		return lim;
	}
	if (x < - lim) {
		return - lim;
	}
	return x;

}