#ifndef _MAIN_H
#define _MAIN_H


#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
///////////////////////////////
enum Mode{mode_send_position1, mode_send_position2, mode_send_position3, mode_error, mode_sethome, mode_complete_home1,mode_complete_home2,mode_complete_home3};
void CAN_Config(void);
void CanWriteData(double data,uint8_t mode);
///////////////////////////////
void RCC_configuration (void);
void Send_Point_Refer(void);
void Config_IO(void);
void Config_Proximity(void);

void Config_send_PC(void);
void Config_send_driver(void);
void USART_DMA_Config(unsigned int BaudRate);
void USART_EVAL_Config(unsigned int BaudRate);
void Send_data_eval(void);
//////////////////////////////////////////////////
struct planning paramsPoly_inverse(double P0[3], double Pf[3], double P0dot[3], double Pfdot[3], double t0, double tf);
double abs_d(double a);
double * Inverse_kinematic(double X, double Y, double Z);
struct planning
{
	double a0[3], a1[3], a2[3], a3[3], a4[3], a5[3];
};

void delay(uint32_t tui);
////////////////////////////////////////////////////
#endif