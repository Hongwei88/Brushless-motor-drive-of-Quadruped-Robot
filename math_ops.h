/*常用的数学计算函数*/
#ifndef MATH_OPS_H
#define MATH_OPS_H

#define PI 3.14159265359f
#define SQRT3 1.73205080757f

#include "math.h"

float fmaxf(float x, float y);						//输出两个数x,y的最大值
float fminf(float x, float y);						//输出两个数x,y的最小值
float fmaxf3(float x, float y, float z);	//输出三个数x,y，z的最大值
float fminf3(float x, float y, float z);	//输出三个数x,y,z的最小值
float roundf(float x);										//返回最接近的整数
void limit_norm(float *x, float *y, float limit);	//将向量(x,y)的长度缩放为<=	limit	
void limit(float *x, float min, float max);				
int float_to_uint(float x, float x_min, float x_max, int bits);		  //转换类型，将无符号整数转换为给定范围的浮点数 
float uint_to_float(int x_int, float x_min, float x_max, int bits); //转换类型，将无符号整数转换为给定范围的浮点数 

#endif
