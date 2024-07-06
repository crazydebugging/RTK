// CoordTrans.cpp : 此文件包含坐标转换函数。
//
#include "RTK_Structs.h"

/****************************************************************************
  XYZToBLH

  目的：笛卡尔坐标转大地坐标

  编号：201

  参数:
  xyz[3]		存储坐标xyz，单位为米
  blh[3]		存储坐标blh，单位为弧度和米
  R		        Radius Earth [m]
  F		        Flattening
****************************************************************************/
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F)
{
    double X = xyz[0];
    double Y = xyz[1];
    double Z = xyz[2];

    const double a = R;
    const double f = F;

    const double e2 = 2.0 * f - f * f;

    double S = sqrt(X * X + Y * Y);
    double L = atan2(Y, X);
    double B = 0;
    double N = 0;
    double tempB = atan2(Z, S);

    int counter = 0;
    do
    {
        B = tempB;
        N = a / sqrt(1 - e2 * sin(B) * sin(B));
        tempB = atan2(Z + N * e2 * sin(B), S);
        counter++;
    } while (fabs(B - tempB) > 1e-12 && counter < 25);

    blh[0] = B;
    blh[1] = L;
    blh[2] = S / cos(B) - N;
}

/****************************************************************************
  BLHToXYZ

  目的：大地坐标转笛卡尔坐标

  编号：202

  参数:
  BLH[3]		存储坐标blh，单位为弧度和米
  XYZ[3]		存储坐标xyz，单位为米
  R		        Radius Earth [m]
  F		        Flattening
****************************************************************************/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F)
{
    // 使用WGS84椭球参数
    const double a = R;
    const double f = F;

    const double e2 = 2.0 * f - f * f;

    const double lambda = BLH[1]; // 经度
    const double phi = BLH[0];    // 纬度
    const double h = BLH[2];      // 高度

    const double N = a / sqrt(1.0 - e2 * sin(phi) * sin(phi));

    XYZ[0] = (N + h) * cos(phi) * cos(lambda);
    XYZ[1] = (N + h) * cos(phi) * sin(lambda);
    XYZ[2] = (N * (1.0 - e2) + h) * sin(phi);
}

/****************************************************************************
  BLHToNEUMat

  目的：测站地平坐标（ENU）转换矩阵计算函数

  编号：203

  参数:
  Blh	    测站大地坐标
  Mat       转换矩阵

****************************************************************************/
void BLHToNEUMat(const double Blh[], double Mat[])
{
    const double lambda = Blh[1]; // 经度
    const double phi = Blh[0];    // 纬度

    // 计算转换矩阵元素
    Mat[0] = -sin(lambda);
    Mat[1] = cos(lambda);
    Mat[2] = 0.0;
    Mat[3] = -sin(phi) * cos(lambda);
    Mat[4] = -sin(phi) * sin(lambda);
    Mat[5] = cos(phi);
    Mat[6] = cos(phi) * cos(lambda);
    Mat[7] = cos(phi) * sin(lambda);
    Mat[8] = sin(phi);
}

/****************************************************************************
  CompSatElAz

  目的：卫星高度角方位角计算函数

  编号：204

  参数:
  Xre	    测站ECEF坐标
  XrE	    测站大地坐标
  Xs        卫星ECEF坐标
  Elev      卫星高度角
  Azim      卫星方位角

****************************************************************************/
void CompSatElAz(const double Xre[], const double XrE [],const double Xs[], double* Elev, double* Azim)
{
    double R[3],Mat[9];
    for (int i = 0; i < 3; i++) {
        R[i] = Xs[i] - Xre[i];
    }
    BLHToNEUMat(XrE, Mat);
    double temp[3];
    MatrixMultiply(3, 3, 3, 1, Mat, R, temp);
    for (int i = 0; i < 3; i++)
    {
        R[i] = temp[i];
    }
    // 计算卫星的高度角
    double rho = sqrt(R[0] * R[0] + R[1] * R[1]);
    *Elev = atan2(R[2],rho);

    // 计算卫星的方位角
    *Azim = atan2(R[0], R[1]);
    if (*Azim < 0) {
        *Azim += 2 * PAI;
    }
}

/****************************************************************************
  Comp_dEnu

  目的：定位误差计算函数

  编号：205

  参数:
  Xr	    测站ECEF坐标
  X0        测站精准ECEF坐标
  dENH      定位误差

****************************************************************************/
void Comp_dEnu(const double X0[],  const double Xr[], double dENH[])
{
    double R[3], Mat[9];
    for (int i = 0; i < 3; i++) {
        R[i] =  Xr[i]-X0[i] ;
    }
    double BLH[3] = { 0 };
    XYZToBLH(Xr, BLH, R_WGS84, F_WGS84);
    BLHToNEUMat(BLH, Mat);
    MatrixMultiply(3, 3, 3, 1, Mat, R, dENH);
}