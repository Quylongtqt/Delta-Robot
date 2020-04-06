#include "main.h"


#define PWM_MAX (double)700
uint8_t Delay_time = 2;
int32_t Counter=0, Rotary=0;

double  epsilon = 0, phi[6][6], theta[6][6], theta_temp[6][6], L[6][6], P[6][6], phiT[6][6], temp[6][6],temp2[6][6], numerator[6][6], denominator[6][6],phiTPphi=0;  ; 
double  lamda_predict = 100, lamda = 0.98;// The higher lamda the more stable, the lower the faster from 0.95 to 1
double Error = 0, pre_Error=0;
double Set_Position = 0, pre_Set_Position = 0, pre_pre_Set_Position = 0;
double Position = 0, pre_Position = 0, pre_pre_Position = 0, pre_pre_pre_Position = 0, pre_pre_pre_pre_Position = 0, pre_pre_pre_pre_pre_Position = 0;
double out = 0, pre_out = 0, pre_pre_out = 0, pre_pre_pre_out = 0, pre_pre_pre_pre_out = 0, pre_pre_pre_pre_pre_out = 0;
int32_t Output_int = 0;
double s0,s1,s2,s3,t0,t1,t2,r1,r2,r3;
double Error;


uint16_t int32_to_uint16(int32_t a)
{
	if(a<0) a = -a;
	a = (uint16_t)(a);
	return a;
}

void STR_Position(void)
{
	MRC_Design();		
	Estimation_Algorithm();
}

// giai thuat uoc luong thong so doi tuong
void Estimation_Algorithm(void)
{
				/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)
			L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))
			P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda
			theta(k) = theta(k-1) + L(k)*epsilon(k)
			phi(k) = [ -y(k-1) -y(k-2) u(k-1) u(k-2)]^T*/
	switch (Delay_time)
{
	case 0:
	{
		phi[0][0] = -pre_Position;
		phi[1][0] = -pre_pre_Position;
		phi[2][0] = -pre_pre_pre_Position;
		phi[3][0] = pre_out;
		phi[4][0] = pre_pre_out;
		phi[5][0] = pre_pre_pre_out;
	}
	case 1:
	{
		phi[0][0] = -pre_pre_Position;
		phi[1][0] = -pre_pre_pre_Position;
		phi[2][0] = -pre_pre_pre_pre_Position;
		phi[3][0] = pre_pre_out;
		phi[4][0] = pre_pre_pre_out;
		phi[5][0] = pre_pre_pre_pre_out;
		break;}
	case 2:
	{
		phi[0][0] = -pre_pre_pre_Position;
		phi[1][0] = -pre_pre_pre_pre_Position;
		phi[2][0] = -pre_pre_pre_pre_pre_Position;
		phi[3][0] = pre_pre_pre_out;
		phi[4][0] = pre_pre_pre_pre_out;
		phi[5][0] = pre_pre_pre_pre_pre_out;
		break;
	}
	default:
	{
		phi[0][0] = -pre_Position;
		phi[1][0] = -pre_pre_Position;
		phi[2][0] = -pre_pre_pre_Position;
		phi[3][0] = pre_out;
		phi[4][0] = pre_pre_out;
		phi[5][0] = pre_pre_pre_out;		
		break;
	}
		
}
	#define MODEL_IMPROVEMENT
	#ifdef MODEL_IMPROVEMENT
			Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
			Multiply_Matrix(phiT,1,6,theta,6,1,temp);//temp = phiT(k)*theta(k-1)
			epsilon = Position - temp[0][0];
			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
			Multiply_Matrix(P,6,6,phi,6,1,numerator);//(P(k-1)*phi(k))
			Multiply_Matrix(phiT,1,6,P,6,6,temp);//temp = phiT(k)*P(k-1)
			Multiply_Matrix(temp,1,6,phi,6,1,denominator);//phiT(k)*P(k-1)*phi(k)
			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;//(1 + phiT(k)*P(k-1)*phi(k))
			Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			if(lamda>1){lamda = 1;}
			if(lamda<0.98){lamda = 0.98;}
			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
			Multiply_Matrix(L,6,1,temp,1,6,temp2);
			Sub_Matrix(P,temp2,6,6,temp);
			Divide_Matrix_Vector(temp,6,6,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			theta_temp[3][0] = theta[3][0];
			Multiply_Matrix_Vector(L,6,1,epsilon,temp);
			Add_Matrix(theta,temp,6,1,theta);
			#else
			Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/
			Multiply_Matrix(phiT,1,6,theta,6,1,temp);//temp = phiT(k)*theta(k-1)
			epsilon = Position - temp[0][0];
			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
			Multiply_Matrix(P,6,6,phi,6,1,numerator);//(P(k-1)*phi(k))
			Multiply_Matrix(phiT,1,6,P,6,6,temp);//temp = phiT(k)*P(k-1)
			Multiply_Matrix(temp,1,6,phi,6,1,denominator);//phiT(k)*P(k-1)*phi(k)
			denominator[0][0] = denominator[0][0] + lamda;//(1 + phiT(k)*P(k-1)*phi(k))
			Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
			Multiply_Matrix(L,6,1,temp,1,6,temp2);
			Sub_Matrix(P,temp2,6,6,temp);
			Divide_Matrix_Vector(temp,6,6,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			theta_temp[3][0] = theta[3][0];
			Multiply_Matrix_Vector(L,6,1,epsilon,temp);
			Add_Matrix(theta,temp,6,1,theta);
			#endif
			/*Recursion*/
			pre_pre_pre_pre_pre_Position = pre_pre_pre_pre_Position;
			pre_pre_pre_pre_Position = pre_pre_pre_Position;
			pre_pre_pre_Position = pre_pre_Position;
			pre_pre_Position = pre_Position;
			pre_Position = Position;

			pre_pre_pre_pre_pre_out = pre_pre_pre_pre_out;
			pre_pre_pre_pre_out = pre_pre_pre_out;
			pre_pre_pre_out = pre_pre_out;
			pre_pre_out = pre_out;
			pre_out = out;
			
			pre_pre_Set_Position = pre_Set_Position;
			pre_Set_Position = Set_Position;
			if (theta[3][0] == 0) {theta[3][0] = theta_temp[3][0];}
}

void Model_Position(void)
{
/* MODEL1 POT=0% Tqt < 0.02 => w = 200 ; epsilon =1 */
//	r1 = theta[4][0] / theta[3][0];
//	r2 = theta[5][0] / theta[3][0];
//	t0 = 0.227906785196605 / theta[3][0];
//	t1 = 0.169988875141426 / theta[3][0];
//	t2 = 0.001662599815162 / theta[3][0];
//	s0 = (-0.735804282272647 - theta[0][0]) / theta[3][0];
//	s1 = ( 0.135368686638193 - theta[1][0]) / theta[3][0];
//	s2 = (-6.144212353326301e-06 - theta[2][0]) / theta[3][0];
	
	
/* MODEL2 POT=0% Tqt < 0.04 => w = 100 ; epsilon =1 */
	r1 = theta[4][0] / theta[3][0];
	r2 = theta[5][0] / theta[3][0];
	t0 = 0.063912710974342/ theta[3][0];
	t1 = 0.086033396260568 / theta[3][0];
	t2 = 0.003828858212441 / theta[3][0];
	s0 = ( -1.219799266424352 - theta[0][0]) / theta[3][0];
	s1 = ( 0.376052984048371 - theta[1][0]) / theta[3][0];
	s2 = (-0.002478752176666 - theta[2][0]) / theta[3][0];
	
///* MODEL3 POT=0% Tqt < 0.025 => w = 160 ; epsilon =1 */
//	r1 = theta[4][0] / theta[3][0];
//	r2 = theta[5][0] / theta[3][0];
//	t0 = 0.156810641871042/ theta[3][0];
//	t1 = 0.143886514589293 / theta[3][0];
//	t2 = 0.002439708085675 / theta[3][0];
//	s0 = ( -0.898993390862346 - theta[0][0]) / theta[3][0];
//	s1 = ( 0.202197984144846 - theta[1][0]) / theta[3][0];
//	s2 = (-6.772873649084615e-05 - theta[2][0]) / theta[3][0];	
}



void MRC_Design(void) // Base on Estimation_Algorithm this function generate suitable control signal for the motor similar to created model
{
		Model_Position();
		Counter = TIM_GetCounter(TIM3);
		Position = (((double)Rotary)*65535.0 + (double)Counter)*360.0/200000.0;


		out = (t0*Set_Position + t1*pre_Set_Position + t2*pre_pre_Set_Position)
					- (s0*Position + s1*pre_Position + s2*pre_pre_Position) 
					- (r1*pre_out + r2*pre_pre_out);
	
		if(out <= -PWM_MAX) out = -PWM_MAX;
		else if (out>= PWM_MAX) out = PWM_MAX;
	  Output_int = (int32_t)out;
		
		if(Output_int<0)
		{
			TIM_SetCompare1(TIM1,0);
			TIM_SetCompare2(TIM1,(uint32_t)(int32_to_uint16(Output_int)));
		}
		else
		{
			TIM_SetCompare1(TIM1,(uint32_t)(int32_to_uint16(Output_int)));
			TIM_SetCompare2(TIM1,0);
		}
}
