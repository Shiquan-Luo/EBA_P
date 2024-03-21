#include "td.h"
#include <cmath>
	TD::TD(const float&r, const float &h0){
		this->d_x1 = 0;
		this->d_x2 = 0;
		this->r = r;
		this->h0 = h0;
	}
	
	float TD::fst(float x1, float x2, float v,float h)
	{
		float td_y = 0;
		float a0 = 0;
		float a = 0;
		float fhan = 0;
		float d = 0;
		float d0 = 0;
	

		d = this->r*h;
		d0 = h*d;
		td_y = x1 - v + h*x2;
		a0 = sqrt(d*d + 8 * r*abs(td_y));

		if (abs(td_y)>d0)
			a = x2 + 0.5*(a0 - d)*sgn(td_y);
		else
			a = x2 + td_y / h;

		if (abs(a)>d)
			fhan = -r*sgn(a);
		else
			fhan = -r*a/d;
		return(fhan);
	}

	float TD::sgn(float x)
	{
		if (x>0)
			return(1);
		if (x<0)
			return(-1);
		if (x == 0)
			return(0);
	}

	double* TD::TD_cal(float input,double x1,double x2,float h){
		d_x1 = x2;
		d_x2 = fst(x1, x2, input,h);
		output[0]=d_x1;
		output[1]=d_x2;
		return output;
	}
