#include "slidingmode.h"

/*
struct slidingmode {
	// def

	double alpha = 9/11; // alpha belongs to (0,1)

	double x1; //sliprate
	//double x1c = 0.2;
	double x1c = 0.117; //best sliprate
	double s1;// x1-x1c
	// parameters of virtual control volume
	double k1;
	double k2;
	double g1;
	double f1;

	double output;//  define output Pressure

	// constructor function
	slidingmode(double K1,double K2) {};

	int inital(double K1,double K2);
	// update state
	int update(double X1, double G1, double F1);

	int adjust(double K1, double K2);
	// calculate actual control volume ,u
	int forefeed(double X1C);

	double cal();
};

int sgn(double x);

double sat(double x);

double Saturation(double x , double t);
*/


slidingmode::slidingmode(double K1, double K2) {
	k1 = K1;
	k2 = K2;
}

int slidingmode::inital(double K1, double K2) {
	k1 = K1;
	k2 = K2;
	return 0;
}

int slidingmode::adjust(double K1, double K2) {
	k1 = K1;
	k2 = K2;
	return 0;
}


int sgn(double x) {
	if (x > 0) {
		return 1;
	}
	if (x < 0) {
		return -1;
	}
	return 0;
}

double sat(double x) {
	double delta = 0.005;
	double k = 1e-4;
	if (abs(x) > delta) {
		return sgn(x);
	}
	else {
		return k * x;
	}

}

double Saturation(double x,double t) {
	double lim = abs(t);
	if (x > lim) {
		return lim;
	}
	if (x < lim) {
		return lim;
	}
	return x;

}

int slidingmode::forefeed(double X1C) {
	x1c = X1C;
	return 0;
}

int slidingmode::update(double X1,double dX1D, double G1, double F1){
	x1 = X1;
	g1 = G1;
	f1 = F1; 
	dx1_D = dX1D;
    return 0;
}



double slidingmode::cal(){

	s1  = x1 - x1c;
	output = 1 / g1 * (-k1 * sgn(s1) - k2 * pow(abs(s1), alpha) * sgn(s1) - f1 + dx1_D);
	return output;

}



