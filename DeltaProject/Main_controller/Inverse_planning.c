#include "main.h"

/*----------parameters-----------*/
#define pi (double)3.141592654
const double alpha[3] = {0, 2*3.141592654/3, 4*3.141592654/3};
#define La (double)200.0
#define	Lb (double)625
#define r (double)60
/*-------------------------------*/
	

struct planning paramsPoly_inverse(double P0[3], double Pf[3], double P0dot[3], double Pfdot[3], double t0, double tf)
{
	struct planning planning_temp;
	for(int8_t i=0; i<3; i++)
	{
//		planning_temp.a0[i] = P0[i];
//		planning_temp.a1[i] = P0dot[i];
//		planning_temp.a2[i] = (-3*(P0[i] - Pf[i]) - (2*P0dot[i] + Pfdot[i])*(tf- t0))/pow((tf-t0),2);
//		planning_temp.a3[i] = (2*(P0[i] - Pf[i]) + (P0dot[i] + Pfdot[i])*(tf- t0))/pow((tf-t0),3);
//		planning_temp.a4[i] = 0;
//		planning_temp.a5[i] = 0;
		planning_temp.a0[i] = P0[i];
		planning_temp.a1[i] = 0;
		planning_temp.a2[i] = 0;
		planning_temp.a3[i] = 10*(Pf[i] - P0[i])/pow((tf-t0),3);
		planning_temp.a4[i] = -15*(Pf[i] - P0[i])/pow((tf-t0),4);
		planning_temp.a5[i] = 6*(Pf[i] - P0[i])/pow((tf-t0),5);
	} 
	return planning_temp;
}	

double * Inverse_kinematic(double X, double Y, double Z)
{
	static double  k=0;
	double thetai1 =0, thetai2 =0;
	static double A[3], B[3], C[3];
	static double theta[3] = {0,0,0};
	static double pre_theta[3] = {0,0,0};

	for(int8_t i=0; i<3; i++)
	{
		A[i] = pow(X,2) + pow(Y,2) + pow(Z,2) + pow(r,2) + pow(La,2) - pow(Lb,2) - 2*r*(X*cos(alpha[i]) + Y*sin(alpha[i]));
		B[i] = -2*La*(-r + X*cos(alpha[i]) + Y*sin(alpha[i]));
		C[i] = -2*La*Z;  
		
		k = 4*pow(C[i],2) - 4*(pow(A[i],2) - pow(B[i],2));
	
		if(k>0 && A[i] != B[i])
		{
			thetai1 = 2*atan((-2*C[i] + sqrt(k))/(2*(A[i] - B[i])));
			thetai2 = 2*atan((-2*C[i] - sqrt(k))/(2*(A[i] - B[i])));

			if(abs_d(thetai1) <= pi/3)
			{	
				theta[i] = thetai1*180/pi;
			}
			else if(abs_d(thetai2) <= pi/3)
			{
				theta[i] = thetai2*180/pi;
			}
		}
		else theta[i] = pre_theta[i];

	}
//	if(theta[0] > 60) theta[0] = 60;
//	if(theta[0] < -60) theta[0] = -60;
//	if(theta[1] > 60) theta[1] = 60;
//	if(theta[1] < -60) theta[1] = -60;
//	if(theta[2] > 60) theta[2] = 60;
//	if(theta[2] < -60) theta[2] = -60;
	
	pre_theta[0] = theta[0];
	pre_theta[1] = theta[1];
	pre_theta[2] = theta[2];
	
	return theta;
}

double abs_d(double a)
{
	if(a<0)
	{
	a = -a;
	}
	return a;
}