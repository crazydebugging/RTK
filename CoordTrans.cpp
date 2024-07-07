// CoordTrans.cpp : ���ļ���������ת��������
//
#include "RTK_Structs.h"

/****************************************************************************
  XYZToBLH

  Ŀ�ģ��ѿ�������ת�������

  ��ţ�201

  ����:
  xyz[3]		�洢����xyz����λΪ��
  blh[3]		�洢����blh����λΪ���Ⱥ���
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

  Ŀ�ģ��������ת�ѿ�������

  ��ţ�202

  ����:
  BLH[3]		�洢����blh����λΪ���Ⱥ���
  XYZ[3]		�洢����xyz����λΪ��
  R		        Radius Earth [m]
  F		        Flattening
****************************************************************************/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F)
{
    // ʹ��WGS84�������
    const double a = R;
    const double f = F;

    const double e2 = 2.0 * f - f * f;

    const double lambda = BLH[1]; // ����
    const double phi = BLH[0];    // γ��
    const double h = BLH[2];      // �߶�

    const double N = a / sqrt(1.0 - e2 * sin(phi) * sin(phi));

    XYZ[0] = (N + h) * cos(phi) * cos(lambda);
    XYZ[1] = (N + h) * cos(phi) * sin(lambda);
    XYZ[2] = (N * (1.0 - e2) + h) * sin(phi);
}

/****************************************************************************
  BLHToNEUMat

  Ŀ�ģ���վ��ƽ���꣨ENU��ת��������㺯��

  ��ţ�203

  ����:
  Blh	    ��վ�������
  Mat       ת������

****************************************************************************/
void BLHToNEUMat(const double Blh[], double Mat[])
{
    const double lambda = Blh[1]; // ����
    const double phi = Blh[0];    // γ��

    // ����ת������Ԫ��
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

  Ŀ�ģ����Ǹ߶ȽǷ�λ�Ǽ��㺯��

  ��ţ�204

  ����:
  Xre	    ��վECEF����
  XrE	    ��վ�������
  Xs        ����ECEF����
  Elev      ���Ǹ߶Ƚ�
  Azim      ���Ƿ�λ��

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
    // �������ǵĸ߶Ƚ�
    double rho = sqrt(R[0] * R[0] + R[1] * R[1]);
    *Elev = atan2(R[2],rho);

    // �������ǵķ�λ��
    *Azim = atan2(R[0], R[1]);
    if (*Azim < 0) {
        *Azim += 2 * PAI;
    }
}

/****************************************************************************
  Comp_dEnu

  Ŀ�ģ���λ�����㺯��

  ��ţ�205

  ����:
  Xr	    ��վECEF����
  X0        ��վ��׼ECEF����
  dENH      ��λ���

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