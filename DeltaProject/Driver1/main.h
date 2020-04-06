#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f10x.h"
#include <stdio.h> 
#include <stdint.h> 
uint16_t int32_to_uint16(int32_t a);

void Config_Pro_LS_IO(void);
void Config_Proximity(void);
void Config_LS(void);
void Config_IO(void);

enum Mode{mode_send_position=0, mode_error};

void Config_RCC(void);
void TIM3_Config_Encoder(void);
void TIM1_Config_PWM(void);
void TIM2_Config_Sample_time(void);
void delay(uint32_t t);

void Can_config(void);
void CanWriteData(double data,uint8_t mode);

void Initial_Condition_Position(void);
void Display_Matrix( double x[6][6], unsigned char m, unsigned char n); // Display matrix to debug
void Multiply_Matrix( double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6]);
void Add_Matrix( double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);
void Sub_Matrix( double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);
void Multiply_Matrix_Vector( double x[6][6],unsigned char m, unsigned char n,double y,  double z[6][6]);
void Divide_Matrix_Vector( double x[6][6],unsigned char m, unsigned char n,double y,  double z[6][6]);
void Tranpose_Matrix( double x[6][6],unsigned char m, unsigned char n,  double y[6][6]);


void Estimation_Algorithm(void);
void MRC_Design(void);
void Model_Position(void);
void STR_Position(void);
#endif