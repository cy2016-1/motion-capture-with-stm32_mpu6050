#include "quaternion.h"
#include <stdio.h>
#include <stdlib.h>

// 实现四元数乘法
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2)
{
    Quaternion result;

    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

    return result;
}

//前置函数 绝对值函数 
float _Abs(float value) {
	if (value < 0)
		return -value;
	else {
		return value;
	}
}

//前置函数 获取矩阵对应元素地址 
float* _Get(void* mat, int i, int j, int N) {
	return ((float*)mat + i * N) + j;
}

//前置函数 行变换 交换矩阵的第a行与第b行 
void _MatSwap(void* mat, int a, int b, int N) {
	float temp;

	for (int i = 0; i < N; i++) {
		temp = *_Get(mat, a, i, N);

		*_Get(mat, a, i, N) = *_Get(mat, b, i, N);

		*_Get(mat, b, i, N) = temp;
	}
}
void _VecSwap(float* vec, int a, int b, int N) {
	float temp;

	temp = vec[a];

	vec[a] = vec[b];

	vec[b] = temp;
}
void _Swap(void* mat, float* vec, int a, int b, int N) {
	_MatSwap(mat, a, b, N);
	_VecSwap(vec, a, b, N);
}

//前置函数 行变换 将矩阵的第a行的k倍加到第b行 
void _MatIk2J(void* mat, int a, float k, int b, int N) {
	float temp;

	for (int i = 0; i < N; i++) {
		temp = *_Get(mat, a, i, N) * k;

		*_Get(mat, b, i, N) += temp;
	}
}
void _VecIk2J(float* vec, int a, float k, int b, int N) {
	float temp;

	temp = k * vec[a];

	vec[b] += temp;
}
void _Ik2J(void* mat, float* vec, int a, float k, int b, int N) {
	_MatIk2J(mat, a, k, b, N);
	_VecIk2J(vec, a, k, b, N);
}


//高斯列主消元法 函数主体 
float* EMCP(void* mat, float* vec, int N) {
	//消元 	
	for (int i = 0; i < N - 1; i++) {
		//寻找列主元 
		int maxEle = i;//列主元索引 
		for (int k = i + 1; k < N; k++) {
			if (_Abs(*_Get(mat, k, i, N)) > _Abs(*_Get(mat, maxEle, i, N)))
				maxEle = k;
		}
		if (*_Get(mat, i, i, N) == 0)
			return NULL;//某列主元为0，方程无解，返回NULL 
		else {
			_Swap(mat, vec, maxEle, i, N);//行交换 
		}

		for (int j = i + 1; j < N; j++) {
			//将列主元下的元素全部消为0	
			_Ik2J(mat, vec, i, -((*_Get(mat, j, i, N)) / (*_Get(mat, i, i, N))), j, N);
		}
	}

	float* v_out = (float*)malloc(sizeof(float) * N);

	//回代 
	for (int i = N - 1; i >= 0; i--) {
		//解出最下方可解的元 
		v_out[i] = vec[i] / (*_Get(mat, i, i, N));

		//对于已经解出的元，从vec中消去该元 
		for (int j = i - 1; j >= 0; j--) {
			vec[j] -= v_out[i] * (*_Get(mat, j, i, N));
		}
	}

	return v_out;
}

void quaternion_inverseTransformation(Quaternion *My_quat)
{
	float _mat[4][4] = {
		/* 初始化系数矩阵 A 的元素 */
		{ My_quat->w, -My_quat->x, -My_quat->y, -My_quat->z},
		{ My_quat->x, My_quat->w, -My_quat->z, My_quat->y},
		{ My_quat->y, My_quat->z, My_quat->w, -My_quat->x},
		{ My_quat->z, -My_quat->y, My_quat->x, My_quat->w}
	};
	float _vec[4] = { 1, 0, 0, 0 };
/*
	Solving Non-homogeneous System of Four Linear Equations
*/
	float* v_out = EMCP(_mat, _vec, 4);
	if(v_out)
	{
		My_quat->w = v_out[0];
		My_quat->x = v_out[1];
		My_quat->y = v_out[2];
		My_quat->z = v_out[3];
	}
//	if (v_out) {
//		printf("Output Vector x:\r\n");
//		for (int i = 0; i < 4; i++) {
//			printf("%.5lf  ", v_out[i]);
//		}
//		printf("\r\n");
//	}
//	else {
//		printf("No Solution!\r\n");
//	}
}
