// Matrix_operation.cpp : 此文件包含矩阵运算函数。
//
#include "RTK_Structs.h"

//文件输出矩阵函数
void outputMatrixToFile(double* matrix, int rows, int cols, const char* filename) {
	FILE* file;
	errno_t err;

	if ((err = fopen_s(&file, filename, "w")) != 0) {
		printf("Failed to open file. Error code: %d\n", err);
		return;
	}

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			fprintf(file, "%10.6f\t", *(matrix + i * cols + j));
		}
		fprintf(file, "\n");
	}

	fclose(file);
}
/****************************************************************************
  compressMatrix

  目的：矩阵压缩(行列指定压缩）

  编号：102

  参数:
  rowsToDelete，colsToDelete      被压缩矩阵指定的压缩行和列
  originalMatrix      输入被压缩矩阵
  originalRows, originalCols		  被压缩矩阵初始行列数
  compressedMatrix		  输出压缩后矩阵

****************************************************************************/
void compressMatrix(int rowsToDelete, int colsToDelete, const double* originalMatrix, int originalRows, int originalCols, double* compressedMatrix)
{
	// 如果没有指定要删除的行或列，即行或列的值小于1或大于原始矩阵的行数或列数，则保持原始矩阵的大小。如果指定了要删除的行或列，则相应的维度减1。
	//根据是否指定删除的行或列来计算压缩后的矩阵的行数和列数。
	int compressedRows = originalRows - (rowsToDelete > 0 && rowsToDelete <= originalRows);
	int compressedCols = originalCols - (colsToDelete > 0 && colsToDelete <= originalCols);
	//对于指定要删除的行或列的情况，我们通过设置偏移量来跳过相应的行或列。
	int rowOffset = 0;
	for (int i = 0; i < originalRows; i++) {
		if (i != rowsToDelete - 1) {
			int colOffset = 0;
			for (int j = 0; j < originalCols; j++) {
				if (j != colsToDelete - 1) {
					compressedMatrix[(i - rowOffset) * compressedCols + (j - colOffset)] = originalMatrix[i * originalCols + j];
				}
				else {
					colOffset = 1;
				}
			}
		}
		else {
			rowOffset = 1;
		}
	}
}
/****************************************************************************
  MatrixMultiply

  目的：矩阵乘法

  编号：103

  参数:
  m1、n1      M1的行数和列数
  m2、n2      M2的行数和列数
  M1		  输入矩阵1
  M2		  输入矩阵2
  M3		  输出矩阵
  返回值：ture=正常，false=致命错误

****************************************************************************/
bool MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[])
{
	if (n1 != m2)
	{
		printf("Error dimension in MatrixInv!\n");
		return false;
	}

	for (int i = 0; i < m1; i++)
	{
		for (int j = 0; j < n2; j++)
		{
			double sum = 0;
			for (int k = 0; k < n1; k++)
			{
				sum += M1[i * n1 + k] * M2[k * n2 + j];
				M3[i * n2 + j] = sum;
			}
		}
	}
	return true;
}
/****************************************************************************
  MatrixMultiply_APB

  目的：矩阵乘法APB

  编号：105

  参数:
  M1		  输入矩阵
  m1、n1      M1的行数和列数
  m2、n2      M2的行数和列数
  P			  权阵
  M2		  输入矩阵
  M3		  输出矩阵
  返回值：1=正常，0=致命错误

****************************************************************************/
bool MatrixMultiply_APB(int m1, int n1, int m2, int n2, const double M1[], const double P[], const double M2[], double M3[])
{
	if (n1 != m2 || m1 <= 0 || n1 <= 0 || m2 <= 0 || n2 <= 0) {
		return false; // Invalid input dimensions
	}

	// Initialize M3 to all zeros
	for (int i = 0; i < m1 * n2; i++) {
		M3[i] = 0.0;
	}

	// Perform APB operation
	for (int i = 0; i < m1; i++) {
		for (int j = 0; j < n2; j++) {
			for (int k = 0; k < n1; k++) {
				M3[i * n2 + j] += M1[i * n1 + k] * P[k] * M2[k * n2 + j];
			}
		}
	}

	return true;
}
/****************************************************************************
  MatrixMultiply_APPB

  目的：矩阵乘法APB(但P是非对角阵)

  编号：105

  参数:
  A		  输入矩阵
  m1、n1      M1的行数和列数
  m2、n2      M2的行数和列数
  P			  权阵
  B		  输入矩阵
  C		  输出矩阵
  返回值：1=正常，0=致命错误

****************************************************************************/
bool MatrixMultiply_APPB(int m1, int n1, int m2, int n2, const double* A, const double* P, const double* B, double* C) {
	if (n1 != m2 || m1 <= 0 || n1 <= 0 || m2 <= 0 || n2 <= 0) {
		return false; // Invalid input dimensions
	}
	// Perform APB operation with optimized memory access
	#pragma omp parallel for  // 使用OpenMP进行并行计算
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < n2; ++j) {
			double sum = 0.0;
			for (int k = 0; k < n1; ++k) {
				double temp = A[i * n1 + k];
				for (int l = 0; l < m2; ++l) {
					sum += temp * P[k * m2 + l] * B[l * n2 + j];
				}
			}
			C[i * n2 + j] = sum;
		}
	}

	return true;
}
/****************************************************************************
  MatrixMultiply_ATPPA

  目的：矩阵乘法ATPPA(但P是非对角阵)

  编号：105

  参数:
  A		  输入矩阵
  m1、n1      A的行数和列数
  n			  权阵的秩
  P			  权阵
  M2		  输出矩阵
  返回值：true=正常，false=致命错误

****************************************************************************/
bool MatrixMultiply_ATPPA(int m1, int n1, int n, const double A[], const double P[], double M2[]) {
	// 检查维度是否匹配：P的行数n必须等于A的行数m1
	if (m1 != n) {
		// 如果不匹配则无法进行矩阵乘法
		return false;
	}
#pragma omp parallel for
	// 创建临时存储中间结果的矩阵
	double* tempMatrix = new double[m1 * n1];

	// 计算P与A的乘积，结果存储在tempMatrix中
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < n1; ++j) {
			tempMatrix[i * n1 + j] = 0;
			for (int k = 0; k < n; ++k) {
				tempMatrix[i * n1 + j] += P[i * n + k] * A[k * n1 + j];
			}
		}
	}

	// 计算A的转置与tempMatrix的乘积，结果存储在M2中
	for (int i = 0; i < n1; ++i) {
		for (int j = 0; j < n1; ++j) {
			M2[i * n1 + j] = 0;
			for (int k = 0; k < m1; ++k) {
				M2[i * n1 + j] += A[k * n1 + i] * tempMatrix[k * n1 + j];
			}
		}
	}

	// 清理临时矩阵的内存
	delete[] tempMatrix;

	// 如果一切正常，返回true
	return true;
}
/****************************************************************************
  MatrixMultiply_ABAT

  目的：矩阵乘法ABAT(但B是非对角阵)

  编号：105

  参数:
  A		  输入矩阵
  m1、n1      A的行数和列数
  n			  权阵的秩
  B			  某一方阵
  M2		  输出矩阵
  返回值：true=正常，false=致命错误

****************************************************************************/
bool MatrixMultiply_ABAT(int m1, int n1, int n, const double A[], const double B[], double M2[]) {
	// 检查维度是否匹配：A的列数n1必须等于B的行数n
	if (n1 != n) {
		// 如果不匹配则无法进行矩阵乘法
		return false;
	}
#pragma omp parallel for
	// 计算 A * B
	double* AB = new double[m1 * n];
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < n; ++j) {
			AB[i * n + j] = 0;
			for (int k = 0; k < n1; ++k) {
				AB[i * n + j] += A[i * n1 + k] * B[k * n + j];
			}
		}
	}

	// 计算 AB * A^T = M2
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < m1; ++j) {
			M2[i * m1 + j] = 0;
			for (int k = 0; k < n; ++k) {
				M2[i * m1 + j] += AB[i * n + k] * A[j * n1 + k];
			}
		}
	}

	// 释放临时矩阵AB的内存
	delete[] AB;

	return true;
}
/****************************************************************************
  MatrixMultiply_ABAT_C

  目的：矩阵乘法ABAT(但B是非对角阵)然后加C

  编号：105

  参数:
  A、C		  输入矩阵
  m1、n1      A的行数和列数
  m      方阵C的秩
  n			  权阵的秩
  B			  某一方阵
  M2		  输出矩阵
  返回值：true=正常，false=致命错误

****************************************************************************/
bool MatrixMultiply_ABAT_C(int m1, int n1, int n, int m, const double A[], const double B[], const double C[], double M2[]) {
	// 检查维度是否匹配
	if (n1 != n || m != m1) {
		// 如果维度不匹配，则无法进行矩阵乘法
		return false;
	}
#pragma omp parallel for
	// 计算 A * B
	double* AB = new double[m1 * n];
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < n; ++j) {
			AB[i * n + j] = 0;
			for (int k = 0; k < n1; ++k) {
				AB[i * n + j] += A[i * n1 + k] * B[k * n + j];
			}
		}
	}

	// 计算 AB * A^T
	double* ABAT = new double[m1 * m1];
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < m1; ++j) {
			ABAT[i * m1 + j] = 0;
			for (int k = 0; k < n; ++k) {
				ABAT[i * m1 + j] += AB[i * n + k] * A[j * n1 + k];
			}
		}
	}

	// 计算 ABAT + C
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < m1; ++j) {
			M2[i * m1 + j] = ABAT[i * m1 + j] + C[i * m1 + j];
		}
	}

	// 释放临时矩阵AB和ABAT的内存
	delete[] AB;
	delete[] ABAT;

	return true;
}
/****************************************************************************
  MatrixMultiply_ABC

  目的：矩阵乘法ABC

  编号：105

  参数:
  A、B、C		  输入矩阵
  m1、n1      A的行数和列数
  m2、n2      B的行数和列数
  m3、n3      C的行数和列数
  M		  输出矩阵
  返回值：true=正常，false=致命错误

****************************************************************************/
bool MatrixMultiply_ABC(int m1, int n1, int m2, int n2, int m3, int n3, const double A[], const double B[], const double C[], double M[]) {
	// 检查维度是否匹配：A的列数n1必须等于B的行数m2，B的列数n2必须等于C的行数m3
	if (n1 != m2 || n2 != m3) {
		// 如果不匹配则无法进行矩阵乘法
		return false;
	}

#pragma omp parallel for  // 使用并行执行
	// 计算 A * B
	double* AB = new double[m1 * n2];
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < n2; ++j) {
			AB[i * n2 + j] = 0;
			for (int k = 0; k < n1; ++k) {
				AB[i * n2 + j] += A[i * n1 + k] * B[k * n2 + j];
			}
		}
	}

	// 计算 AB * C = M
	for (int i = 0; i < m1; ++i) {
		for (int j = 0; j < n3; ++j) {
			M[i * n3 + j] = 0;
			for (int k = 0; k < n2; ++k) {
				M[i * n3 + j] += AB[i * n2 + k] * C[k * n3 + j];
			}
		}
	}

	// 释放临时矩阵AB的内存
	delete[] AB;

	return true;
}
/****************************************************************************
  MatrixAddition

  目的：两个矩阵加法

  编号：106

  参数:
  M1		  输入矩阵1
  m、n        输入矩阵的行数和列数
  M2		  输入矩阵2
  M3		  输出矩阵

****************************************************************************/
void MatrixAddition(int m, int n, const double M1[], const double M2[], double M3[])
{
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			M3[i * n + j] = M1[i * n + j] + M2[i * n + j];
		}
	}
}
/****************************************************************************
  MatrixAddition2

  目的：两个矩阵加至第二个矩阵

  编号：107

  参数:
  M1		  输入矩阵1
  m、n        输入矩阵的行数和列数
  M2		  输出矩阵

****************************************************************************/
void MatrixAddition2(int m, int n, const double M1[], double M2[])
{
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			M2[i * n + j] += M1[i * n + j];
		}
	}
}
/****************************************************************************
  MatrixSubtraction

  目的：两个矩阵减法

  编号：108

  参数:
  M1		  输入矩阵1
  m、n        输入矩阵的行数和列数
  M2		  输入矩阵2
  M3		  输出矩阵

****************************************************************************/
void MatrixSubtraction(int m, int n, const double M1[], const double M2[], double M3[])
{
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			M3[i * n + j] = M1[i * n + j] - M2[i * n + j];
		}
	}
}
/****************************************************************************
  MatrixTranspose

  目的：矩阵转置

  编号：109

  参数:
  M1		  输入矩阵
  m、n        输入矩阵的行数和列数
  MT		  输出转置矩阵

****************************************************************************/
void MatrixTranspose(int m, int n, const double M1[], double MT[])
{
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			MT[i * m + j] = M1[j * n + i];
		}
	}
}
/****************************************************************************
  MatrixInv

  目的：矩阵求逆,采用全选主元高斯-约当法

  编号：110

  参数:
  n      M1的行数和列数
  a      输入矩阵
  b      输出矩阵   b=inv(a)
  返回值：1=正常，0=致命错误

****************************************************************************/
int MatrixInv(int n, double a[], double b[])
{
	int i, j, k, l, u, v; 
	/* matrix dimension <= 10 */
	int is[50] = { 0 }; // 初始化为0
	int js[50] = { 0 }; // 初始化为0
	double d, p;

	if (n <= 0)
	{
		printf("Error dimension in MatrixInv!\n");
		return 0;
	}

	/* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			b[i * n + j] = a[i * n + j];
		}
	}

	for (k = 0; k < n; k++)
	{
		d = 0.0;
		for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
		{
			for (j = k; j < n; j++)
			{
				l = n * i + j;
				p = fabs(b[l]);
				if (p > d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d < DBL_EPSILON)   /* 主元素接近于0，矩阵不可逆 */
		{
			printf("Divided by 0 in MatrixInv!\n");
			return 0;
		}

		if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = is[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = i * n + js[k];
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		l = k * n + k;
		b[l] = 1.0 / b[l];  /* 初等行变换 */
		for (j = 0; j < n; j++)
		{
			if (j != k)
			{
				u = k * n + j;
				b[u] = b[u] * b[l];
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				for (j = 0; j < n; j++)
				{
					if (j != k)
					{
						u = i * n + j;
						b[u] = b[u] - b[i * n + k] * b[k * n + j];
					}
				}
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				u = i * n + k;
				b[u] = -b[u] * b[l];
			}
		}
	}

	for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
	{
		if (js[k] != k)
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = is[k] + i * n;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
	}

	return (1);
}
// Function to calculate inverse of a matrix using LU decomposition
int lu_decomposition_inverse(int n, double* a, double* b) {
	int i, j, k;
	double* L = (double*)malloc(n * n * sizeof(double));
	double* U = (double*)malloc(n * n * sizeof(double));
	double sum;
	if (n <= 0)
	{
		printf("Error dimension in MatrixInv!\n");
		return 0;
	}
	// 初始化L和U为单位矩阵
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			if (i == j) {
				L[i * n + j] = 1;
				U[i * n + j] = 1;
			}
			else {
				L[i * n + j] = 0;
				U[i * n + j] = 0;
			}
		}
	}

	// 进行LU分解
	for (i = 0; i < n; i++) {
		for (j = i; j < n; j++) {
			sum = 0;
			for (k = 0; k < i; k++) {
				sum += L[i * n + k] * U[k * n + j];
			}
			U[i * n + j] = a[i * n + j] - sum;
		}
		for (j = i + 1; j < n; j++) {
			sum = 0;
			for (k = 0; k < i; k++) {
				sum += L[j * n + k] * U[k * n + i];
			}
			L[j * n + i] = (a[j * n + i] - sum) / U[i * n + i];
		}
	}
	// 检查U对角线元素是否为零
	for (i = 0; i < n; i++) {
		if (fabs(U[i * n + i]) < DBL_EPSILON) {
			printf("Divided by 0 in MatrixInv!\n");
			free(L);
			free(U);
			return 0; // 提前返回，不再继续计算逆矩阵
		}
	}

	// 求解逆矩阵
	for (k = 0; k < n; k++) {
		// Forward substitution for L
		for (i = 0; i < n; i++) {
			sum = (i == k) ? 1.0 : 0.0;
			for (j = 0; j < i; j++) {
				sum -= L[i * n + j] * b[j * n + k];
			}
			b[i * n + k] = sum / L[i * n + i];
		}

		// Backward substitution for U
		for (i = n - 1; i >= 0; i--) {
			sum = b[i * n + k];
			for (j = i + 1; j < n; j++) {
				sum -= U[i * n + j] * b[j * n + k];
			}
			b[i * n + k] = sum / U[i * n + i];
		}
	}

	free(L);
	free(U);

	return 1;
}
/****************************************************************************
  VectorAddition

  目的：该函数将两个输入向量相加，并将结果存储在输出向量中

  编号：111

  参数:
  n      向量的长度
  v1	 第一个输入向量
  v2	 第二个输入向量
  v3	 输出向量，用于存储相加结果
****************************************************************************/
void VectorAddition(int n, const double v1[], const double v2[], double v3[])
{
	for (int i = 0; i < n; i++) {
		v3[i] = v1[i] + v2[i];
	}
}

/****************************************************************************
  VectorSubtraction

  目的：该函数将两个输入向量相减，并将结果存储在输出向量中

  编号：112

  参数:
  n      向量的长度
  v1	 第一个输入向量
  v2	 第二个输入向量
  v3	 输出向量，用于存储相加结果
****************************************************************************/
void VectorSubtraction(int n, const double v1[], const double v2[], double v3[])
{
	for (int i = 0; i < n; i++) {
		v3[i] = v1[i] - v2[i];
	}
}

/****************************************************************************
  VectDot

  目的：向量点积

  编号：113

  参数:
  m      第一个输入向量的长度
  n      第二个输入向量的长度
  A		 第一个输入向量
  B		 第二个输入向量

  返回值：向量点积结果
****************************************************************************/
double VectDot(int m, int n, const double A[], const double B[])
{
	if (m != n) {
		// 向量长度不匹配，无法计算点积
		return 0.0; // 或者根据需求返回适当的错误值
	}

	double dotProduct = 0.0;

	for (int i = 0; i < m; i++) {
		dotProduct += A[i] * B[i];
	}

	return dotProduct;
}

/****************************************************************************
  Norm

  目的：求向量长度

  编号：114

  参数:
  n      输入向量的长度(维度)
  A		 输入向量
  返回值：向量长度
****************************************************************************/
double Norm(const int n, const double A[])
{
	double norm = 0.0;

	for (int i = 0; i < n; i++) {
		norm += A[i] * A[i];
	}

	return sqrt(norm);
}

/****************************************************************************
  CrossDot

  目的：向量叉积

  编号：115

  参数:
  m      第一个输入向量的长度
  n      第二个输入向量的长度
  A		 第一个输入向量
  B		 第二个输入向量
  C		 输出向量
****************************************************************************/
void CrossDot(int m, int n, const double A[], const double B[], double C[])
{
	if (m != n || m != 3) {
		// 只有长度为3的向量才能进行叉积计算
		return; // 或者根据需求执行适当的错误处理
	}

	C[0] = A[1] * B[2] - A[2] * B[1];
	C[1] = A[2] * B[0] - A[0] * B[2];
	C[2] = A[0] * B[1] - A[1] * B[0];
}

/****************************************************************************
  AdjRow

  目的：矩阵加数

  编号：118

  参数:
  M			  运算矩阵
  m、n        输入矩阵的行数和列数
  isAdd		  已经添加的个数
  row		  要存放数据的行数
  val		  要输入的值

****************************************************************************/
void AdjRow(int m, int n, int isAdd, int row, double val, double M[])
{
	// 计算当前行在一维矩阵中的起始索引及其值
	int Index = (row-1) * n+isAdd;
	// 存放数据的位置
	M[Index] = val;
}