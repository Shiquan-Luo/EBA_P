#pragma once
#ifndef TD_H
#define TD_H

class TD {

public:
	//TD parameters
	float r;
	float h;
	float h0;
	float x1;
	float x2;
	float d_x1;
	float d_x2;
	double output[2];
public:
	TD(const float&r, const float &h0);
	float fst(float x1, float x2, float v,float h);
	float sgn(float x);
	double* TD_cal(float input, double x1, double x2, float h);
	};

#endif // TD_H
