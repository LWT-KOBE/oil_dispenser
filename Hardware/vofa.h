#ifndef __VOFA_H
#define __VOFA_H 			   
#include "system.h"  
	 
void RawData_Test(void);
void FireWater_Test(void);
void Float_to_Byte(float f,unsigned char byte[]);
void Byte_to_Float(float *f,unsigned char byte[]);
void JustFloat_Test(void);

void Float_to_Byte(float f,unsigned char byte[]);
void Vofa_sendData(float a, float b, float c);

void vofa_sendData(float a, float b, float c, float d, float e, float f, float g, float h, float j, float k, float l, float o, float p, float i,float y,float t,float r);
#endif





























