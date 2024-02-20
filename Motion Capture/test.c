#include <stdio.h>
#include <stdlib.h>

/*
	————————————————
	说明：
	————————————————
	高斯列主消元法主函数为 EMCP()
	函数原型为：
	Double * EMCP(void * mat,float * vec)
	具体用法见main()函数

	使用时将main函数前的所有函数复制到自己程序的main函数前面
	根据main中说明的使用方法使用本函数

	复制时 所有前置函数 缺一不可，且顺序不可随意调换
	最后一个PrintMat()是用来打印二维数组的，可要可不要
	————————————————
	@猿力觉醒
*/

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
		if (*_Get(mat, i, i, N) == 0.0)
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

//打印输出矩阵 
void PrintMat(void* mat, int N) {
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			printf("%.2lf", *_Get(mat, i, j, N));
			printf("  ");
		}
		printf("\n\n");
	}
}


#define N 4

typedef struct {
	float w, x, y, z;
} Quaternion;

Quaternion My_quat = { 0.707, 0.707, 0, 0 };

//以下为测试用main函数，示意函数使用方法 
int main(void) {
	float _mat[N][N] = {
		/* 初始化系数矩阵 A 的元素 */
		{ My_quat.w, -My_quat.x, -My_quat.y, -My_quat.z},
		{ My_quat.x, My_quat.w, -My_quat.z, My_quat.y},
		{ My_quat.y, My_quat.z, My_quat.w, -My_quat.x},
		{ My_quat.z, -My_quat.y, My_quat.x, My_quat.w}
	};

	//	EMCP()函数会更改传入的系数矩阵mat与非齐次项vec 
	//	故此处为了演示用法重定义了一模一样的mat与vec 
	float _vec[N] = { 1, 0, 0, 0 };

	//【关键处】 
	//下面的语句展示了调用EMCP的用法 	
	//	float * EMCP(mat,vec,N) 一共需要3个参数
	//	第一个参数mat为系数矩阵，传入C语言中的二维数组的名字即可
	//	第二个参数vec为非齐次项的列向量，传入C语言中的一维数组名字即可 
	//	第三个参数N为方程的阶数
	//
	//	使用时必须保证mat与vec维数一致 
	float* v_out = EMCP(_mat, _vec, N);
	//EMCP()返回一个双精度一维数组 
	//	即解向量数组
	//	上面 v_out 与一维数组等价 
	//	可以使用如 v_out[i] 的形式访问每个元素 
	//

	printf("A' is:\n");
	PrintMat(_mat, N);
	printf("\n");

	printf("b' = \n");
	for (int i = 0; i < N; i++) {
		printf("%.2lf  ", _vec[i]);
	}
	printf("\n\n");

	if (v_out) {
		printf("Output Vector x:\n");
		for (int i = 0; i < N; i++) {
			printf("%.5lf  ", v_out[i]);
		}
		printf("\n\n");
	}
	else {
		printf("No Solution!");
	}
}