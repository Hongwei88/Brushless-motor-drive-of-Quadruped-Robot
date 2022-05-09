/*���õ���ѧ���㺯��*/
#ifndef MATH_OPS_H
#define MATH_OPS_H

#define PI 3.14159265359f
#define SQRT3 1.73205080757f

#include "math.h"

float fmaxf(float x, float y);						//���������x,y�����ֵ
float fminf(float x, float y);						//���������x,y����Сֵ
float fmaxf3(float x, float y, float z);	//���������x,y��z�����ֵ
float fminf3(float x, float y, float z);	//���������x,y,z����Сֵ
float roundf(float x);										//������ӽ�������
void limit_norm(float *x, float *y, float limit);	//������(x,y)�ĳ�������Ϊ<=	limit	
void limit(float *x, float min, float max);				
int float_to_uint(float x, float x_min, float x_max, int bits);		  //ת�����ͣ����޷�������ת��Ϊ������Χ�ĸ����� 
float uint_to_float(int x_int, float x_min, float x_max, int bits); //ת�����ͣ����޷�������ת��Ϊ������Χ�ĸ����� 

#endif
