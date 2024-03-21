#ifndef _SLIDINGMODE_H_
#define _SLIDINGMODE_H_

#include <iostream>
#include <cmath>

struct slidingmode {
	// def
	
	double alpha = 9/11; // alpha belongs to (0,1)

	double dx1_D;
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
	slidingmode(double K1,double K2);
	
	int inital(double K1,double K2);
	// update state
	int update(double X1, double dX1D, double G1, double F1);
	
	int adjust(double K1, double K2);
	// calculate actual control volume ,u
	int forefeed(double X1C);

	double cal();
};

int sgn(double x);

double sat(double x);

double Saturation(double x , double t);

#endif