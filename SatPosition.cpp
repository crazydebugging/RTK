// SatPosition.cpp : ���ļ���������λ�ü��㺯���Ͷ��������ģ�͡�
//
#include "RTK_Structs.h"

#define GPSEPHTIMELIMIT 7500
#define	BDSEPHTIMELIMIT 3900
#define F -4.442807633e-10

void Rotation_z(double t, double* Mat)
{
	Mat[0] = cos(t); Mat[1] = sin(t); Mat[2] = 0;
	Mat[3] = -sin(t); Mat[4] = cos(t); Mat[5] = 0;
	Mat[6] = 0; Mat[7] = 0; Mat[8] = 1;
}
void Rotation_x(double t, double* Mat)
{
	Mat[0] = 1; Mat[1] = 0; Mat[2] = 0;
	Mat[3] = 0; Mat[4] = cos(t); Mat[5] = sin(t);
	Mat[6] = 0; Mat[7] = -sin(t); Mat[8] =cos(t);
}
void Rotation_zDot(double t, double* Mat)
{
	Mat[0] = -sin(t); Mat[1] = cos(t); Mat[2] = 0;
	Mat[3] = -cos(t); Mat[4] = -sin(t); Mat[5] = 0;
	Mat[6] = 0; Mat[7] = 0; Mat[8] = 0;
}
void Rotation_xDot(double t, double* Mat)
{
	Mat[0] = 0; Mat[1] = 0; Mat[2] = 0;
	Mat[3] = 0; Mat[4] = -sin(t); Mat[5] = cos(t);
	Mat[6] = 0; Mat[7] = -cos(t); Mat[8] = -sin(t);
}
// �㲥����
/****************************************************************************
  CompSatClkOff

  Ŀ�ģ�GPS��BDS�����Ӳ������ټ���

  ��ţ�401

  ����:
  Prn			�������к�
  Sys			����ϵͳ��
   t		    �źŷ���ʱ��
  GPSEph		gps��������
  BDSEph		bds��������
  Mid			ÿ������λ�á��ٶȺ��Ӳ�ȵ��м������

  attention:ע��BDS�����Ĳο�ʱ��ΪBDT��GPST-BDT=14s;

  ����ֵ�����������жϣ������������Health�Ƿ�Ϊ0��������ڻ��߲�����������false�����򷵻�true
****************************************************************************/
bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATMIDRES* Mid)
{
	double dt, limt;
	GPSEPHREC* eph;
	GPSTIME BT;

	eph = (Sys == GPS) ? GPSEph + Prn - 1 : BDSEph + Prn - 1;
	if (eph->PRN != Prn || eph->Sys != Sys || eph->SVHealth) return false;

	limt= (Sys == GPS) ? GPSEPHTIMELIMIT : BDSEPHTIMELIMIT;
	BT.SecOfWeek = eph->TOE.SecOfWeek + 14;
	BT.Week = eph->TOE.Week + 1356;
	dt = (Sys == GPS) ? (t->Week - eph->TOC.Week) * 604800 + (t->SecOfWeek - eph->TOC.SecOfWeek) : GetDiffTime(t, &BT);
	if (fabs(dt) > limt) return false;

	//���������ЧӦ
	double A = eph->SqrtA * eph->SqrtA;
	double n0 = (Sys == GPS) ? sqrt(GM_Earth / (A * A * A)) : sqrt(GM_BDS / (A * A * A));
	double n = n0 + eph->DeltaN;
	double Mk = eph->M0 + n * dt;
	// ��ʼ����������
	double Ek = 0;
	double deltaE = 1.0;
	// ��������
	while (fabs(deltaE) > 3e-9) {
		double temp = Mk + eph->e * sin(Ek);
		deltaE = temp - Ek;
		Ek = temp;
	}
	double EkDot = n / (1 - eph->e * cos(Ek));
	double dtr = F * eph->e * eph->SqrtA * sin(Ek);
	double dtrDot = F * eph->e * eph->SqrtA * cos(Ek)* EkDot;

	Mid->SatClkOft =eph->ClkBias +eph->ClkDrift * dt +eph->ClkDriftRate * dt * dt+dtr;
	Mid->SatClkSft =eph->ClkDrift +eph->ClkDriftRate * dt * 2+ dtrDot;
	Mid->Valid = true;
	return true;
}
/****************************************************************************
  CompGPSSatPVT

  Ŀ�ģ�GPS����λ�ú��ٶ�

  ��ţ�402

  ����:
  Prn			�������к�
   t		    �źŷ���ʱ��
  GPSEph		gps��������
  Mid			ÿ��GPS����λ�á��ٶȺ��Ӳ�ȵ��м������

  ����ֵ������ɹ���ʧ��
****************************************************************************/
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
	const GPSEPHREC* geph =  Eph;
	SATMIDRES* mid = Mid;

	double tk = (t->Week - geph->TOE.Week) * 604800 + (t->SecOfWeek - geph->TOE.SecOfWeek);
	double A = geph->SqrtA * geph->SqrtA;
	double n0 = sqrt(GM_Earth / (A * A * A));
	double n = n0 + geph->DeltaN;
	double Mk = geph->M0 + n * tk;
	// ��ʼ����������
	double Ek = 0;
	double deltaE = 1.0;
		// ��������
	while (fabs(deltaE) > 3e-9) {
		double temp = Mk + geph->e * sin(Ek);
		deltaE = temp - Ek;
		Ek = temp;
	}
	double EkDot = n / (1 - geph->e * cos(Ek));
	double vk = atan2(sqrt(1 - geph->e * geph->e) * sin(Ek), cos(Ek) - geph->e);
	double Phik = vk + geph->omega;
	double PhikDot = sqrt((1+ geph->e)/ (1 - geph->e))*pow(cos(vk/2)/ cos( Ek / 2),2)* EkDot;
	double duk = geph->Cus * sin(2 * Phik) + geph->Cuc * cos(2 * Phik);
	double drk = geph->Crs * sin(2 * Phik) + geph->Crc * cos(2 * Phik);
	double dik = geph->Cis * sin(2 * Phik) + geph->Cic * cos(2 * Phik);
	double uk = Phik + duk;
	double ukDot = 2*(geph->Cus * cos(2 * Phik) - geph->Cuc * sin(2 * Phik))*PhikDot + PhikDot;
	double rk = A * (1 - geph->e * cos(Ek)) + drk;
	double rkDot = A * geph->e* sin(Ek)* EkDot+2* (geph->Crs * cos(2 * Phik) - geph->Crc * sin(2 * Phik))* PhikDot;
	double ik = geph->i0 + dik + geph->iDot * tk;
	double ikDot = geph->iDot+ 2*(geph->Cis * cos(2 * Phik) - geph->Cic * sin(2 * Phik)) * PhikDot;
	double xk0 = rk * cos(uk);
	double xk0Dot = rkDot * cos(uk)-rk*ukDot*sin(uk);
	double yk0 = rk * sin(uk);
	double yk0Dot = rkDot * sin(uk) + rk * ukDot * cos(uk);
	double Omegak = geph->OMEGA + (geph->OMEGADot - Omega_WGS) * tk - Omega_WGS * geph->TOE.SecOfWeek;
	double OmegakDot = geph->OMEGADot - Omega_WGS;
	double R[12] = { 0 };
	R[0] = cos(Omegak); R[1] = -sin(Omegak)*cos(ik); R[2] = -(xk0*sin(Omegak) + yk0 * cos(ik) * cos(Omegak)); R[3] = yk0 * sin(ik) * sin(Omegak);
	R[4] = sin(Omegak); R[5] = cos(Omegak) * cos(ik); R[6] = xk0 * cos(Omegak) - yk0 * cos(ik) * sin(Omegak); R[7] = -yk0 * sin(ik) * cos(Omegak);
	R[8] = 0; R[9] =  sin(ik); R[10] = 0; R[11] = yk0 * cos(ik);

	mid->SatPos[0] = xk0 * cos(Omegak) - yk0 * cos(ik) * sin(Omegak);
	mid->SatPos[1] = xk0 * sin(Omegak) + yk0 * cos(ik) * cos(Omegak);
	mid->SatPos[2] = yk0 * sin(ik);

	double temp[4] = { 0 };
	temp[0] = xk0Dot; temp[1] = yk0Dot; temp[2] = OmegakDot; temp[3] = ikDot;
	MatrixMultiply(3, 4, 4, 1, R, temp, mid->SatVel);
	return 1;
}

/****************************************************************************
  CompBDSSatPVT

  Ŀ�ģ�BDS����λ�á��ٶ�

  ��ţ�403

  ����:
  Prn			�������к�
   t		    �źŷ���ʱ��
  BDSEph		bds��������
  Mid			ÿ��BDS����λ�á��ٶȺ��Ӳ�ȵ��м������

  ����ֵ������ɹ���ʧ��
****************************************************************************/
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
	const GPSEPHREC* beph =  Eph;
	SATMIDRES* mid =  Mid;
	double dt = 1e-4;
	GPSTIME BT;
	BT.SecOfWeek = beph->TOE.SecOfWeek + 14;
	BT.Week = beph->TOE.Week + 1356;
	double tk = GetDiffTime(t, &BT);

	double A = beph->SqrtA * beph->SqrtA;
	double n0 = sqrt(GM_BDS / (A * A * A));
	double n = n0 + beph->DeltaN;
	double Mk = beph->M0 + n * tk;
	// ��ʼ����������
	double Ek = 0;
	double deltaE = 1.0;
	// ��������
	while (fabs(deltaE) > 3e-9) {
		double temp = Mk + beph->e * sin(Ek);
		deltaE = temp - Ek;
		Ek = temp;
	}
	double EkDot = n / (1 - beph->e * cos(Ek));
	double vk = atan2(sqrt(1 - beph->e * beph->e) * sin(Ek), cos(Ek) - beph->e);
	double Phik = vk + beph->omega;
	double PhikDot = sqrt((1 + beph->e) / (1 - beph->e)) * pow(cos(vk / 2) / cos(Ek / 2), 2) * EkDot;
	double duk = beph->Cus * sin(2 * Phik) + beph->Cuc * cos(2 * Phik);
	double drk = beph->Crs * sin(2 * Phik) + beph->Crc * cos(2 * Phik);
	double dik = beph->Cis * sin(2 * Phik) + beph->Cic * cos(2 * Phik);
	double uk = Phik + duk;
	double ukDot = 2 * (beph->Cus * cos(2 * Phik) - beph->Cuc * sin(2 * Phik)) * PhikDot + PhikDot;
	double rk = A * (1 - beph->e * cos(Ek)) + drk;
	double rkDot = A * beph->e * sin(Ek) * EkDot + 2 * (beph->Crs * cos(2 * Phik) - beph->Crc * sin(2 * Phik)) * PhikDot;
	double ik = beph->i0 + dik + beph->iDot * tk;
	double ikDot = beph->iDot + 2 * (beph->Cis * cos(2 * Phik) - beph->Cic * sin(2 * Phik)) * PhikDot;
	double xk0 = rk * cos(uk);
	double xk0Dot = rkDot * cos(uk) - rk * ukDot * sin(uk);
	double yk0 = rk * sin(uk);
	double yk0Dot = rkDot * sin(uk) + rk * ukDot * cos(uk);
	if (beph->i0 > Rad * 30)
	{
		double Omegak = beph->OMEGA + (beph->OMEGADot - Omega_BDS) * tk - Omega_BDS * beph->TOE.SecOfWeek;
		double OmegakDot = beph->OMEGADot - Omega_BDS;
		double R[12] = { 0 };
		R[0] = cos(Omegak); R[1] = -sin(Omegak) * cos(ik); R[2] = -(xk0 * sin(Omegak) + yk0 * cos(ik) * cos(Omegak)); R[3] = yk0 * sin(ik) * sin(Omegak);
		R[4] = sin(Omegak); R[5] = cos(Omegak) * cos(ik); R[6] = xk0 * cos(Omegak) - yk0 * cos(ik) * sin(Omegak); R[7] = -yk0 * sin(ik) * cos(Omegak);
		R[8] = 0; R[9] = sin(ik); R[10] = 0; R[11] = yk0 * cos(ik);

		mid->SatPos[0] = xk0 * cos(Omegak) - yk0 * cos(ik) * sin(Omegak);
		mid->SatPos[1] = xk0 * sin(Omegak) + yk0 * cos(ik) * cos(Omegak);
		mid->SatPos[2] = yk0 * sin(ik);

		double temp[4] = { 0 };
		temp[0] = xk0Dot; temp[1] = yk0Dot; temp[2] = OmegakDot; temp[3] = ikDot;
		MatrixMultiply(3, 4, 4, 1, R, temp, mid->SatVel);
	}
	else
	{
		double Omegak = beph->OMEGA + beph->OMEGADot * tk - Omega_BDS * beph->TOE.SecOfWeek;
		double xyz[3];
		xyz[0] = xk0 * cos(Omegak) - yk0 * cos(ik) * sin(Omegak);
		xyz[1] = xk0 * sin(Omegak) + yk0 * cos(ik) * cos(Omegak);
		xyz[2] = yk0 * sin(ik);
		double RX[9]; Rotation_x(Rad * -5, RX);
		double t = Omega_BDS * tk; 
		double RZ[9];
		Rotation_z(t, RZ);
		double RZX[9];
		MatrixMultiply(3, 3, 3, 3, RZ, RX, RZX);
		MatrixMultiply(3, 3, 3, 1, RZX, xyz, mid->SatPos);

		//���ٶ�
		double OmegakDot = beph->OMEGADot;
		double R[12] = { 0 };
		R[0] = cos(Omegak); R[1] = -sin(Omegak) * cos(ik); R[2] = -(xk0 * sin(Omegak) + yk0 * cos(ik) * cos(Omegak)); R[3] = yk0 * sin(ik) * sin(Omegak);
		R[4] = sin(Omegak); R[5] = cos(Omegak) * cos(ik); R[6] = xk0 * cos(Omegak) - yk0 * cos(ik) * sin(Omegak); R[7] = -yk0 * sin(ik) * cos(Omegak);
		R[8] = 0; R[9] = sin(ik); R[10] = 0; R[11] = yk0 * cos(ik);

		double temp[4] = { 0 };
		temp[0] = xk0Dot; temp[1] = yk0Dot; temp[2] = OmegakDot; temp[3] = ikDot;
		double temp1[3] = { 0 }; double temp2[3] = { 0 }; double RZXR[12] = { 0 };
		MatrixMultiply(3, 3, 3, 4, RZX, R,RZXR); MatrixMultiply(3, 4, 4, 1, RZXR, temp, temp1);

		double RZDot[9]; Rotation_zDot(t, RZDot);
		for (int i = 0; i < 9; i++)
		{
			RZDot[i] = RZDot[i] * Omega_BDS;
		}
		double RZDotX[9]; 
		MatrixMultiply(3, 3, 3, 3, RZDot, RX, RZDotX);
		MatrixMultiply(3, 3, 3, 1, RZDotX, xyz, temp2);

		MatrixAddition(3, 1, temp1, temp2, mid->SatVel);
	}
	return 1;
}
/****************************************************************************
  ComputeGPSSatOrbitAtSignalTrans

  Ŀ�ģ������źŷ���ʱ�̵�����λ���ٶȺ��Ӳ����٣����������ת���������㷽λ�Ǻ͸߶Ƚ��Լ��������ӳٵ�

  ��ţ�404

  ����:
  Epk			�۲����ݽṹ��
  GPSEph		gps��������
  BDSEph		bds��������
  RcvPos		�û���λ���

  ����ֵ������ɹ���ʧ������SatPVT�е�Valid

****************************************************************************/
void ComputeGPSSatOrbitAtSignalTrans(EPOCHOBS* Epk, GPSEPHREC* GpsEph, GPSEPHREC* BDSEph, double RcvPos[3])
{
	int prn;
	GPSTIME T_Tm;  /*�źŷ���ʱ�̵�ϵͳʱ��*/
	double dt0;/*�źŴ���ʱ��*/
	double RotAng=0; /*�źŴ���ʱ���ڵĵ�����ת�Ƕ�[Rad]*/
	double Mat[9];/*������ת��������*/
	double dPos[3];/*�źŴ�������*/

	T_Tm.Week = Epk->Time.Week;
	for (int i = 0; i < Epk->SatNum; i++)
	{
		Epk->SatPVT[i].Valid = 0;  //����ǰ������Ϊ������
		prn = Epk->SatObs[i].Prn;

		T_Tm.SecOfWeek = Epk->Time.SecOfWeek - Epk->ComObs[i].PIF / C_Light;
		if ((Epk->SatPVT[i].Valid = CompSatClkOff(prn, Epk->SatObs[i].System, &T_Tm, GpsEph, BDSEph, &Epk->SatPVT[i])) < 1) continue;

		T_Tm.SecOfWeek = Epk->Time.SecOfWeek - Epk->ComObs[i].PIF / C_Light- Epk->SatPVT[i].SatClkOft;
		if ((Epk->SatPVT[i].Valid = CompSatClkOff(prn, Epk->SatObs[i].System, &T_Tm, GpsEph, BDSEph, &Epk->SatPVT[i])) < 1) continue;

		if (Epk->SatObs[i].System == GPS)
		{
			Epk->SatPVT[i].Valid = CompGPSSatPVT(prn, &T_Tm, GpsEph + prn - 1, &Epk->SatPVT[i]);
		}
		if (Epk->SatObs[i].System == BDS)
		{
			Epk->SatPVT[i].Valid = CompBDSSatPVT(prn, &T_Tm, BDSEph + prn - 1, &Epk->SatPVT[i]);
		}

		if (Epk->SatPVT[i].Valid <= 0) continue;

		MatrixSubtraction(3, 1, Epk->SatPVT[i].SatPos, RcvPos, dPos);
		dt0 = Norm(3, dPos)/ C_Light;
		if (Epk->SatObs[i].System == GPS)
		{
			RotAng = Omega_WGS * dt0;  /*������ת����*/
		}
		if (Epk->SatObs[i].System == BDS)
		{
			RotAng = Omega_BDS * dt0;  /*������ת����*/
		}
		Rotation_z(RotAng, Mat);
		double temp[3];//����������
		bool success=MatrixMultiply(3, 3, 3, 1, Mat, Epk->SatPVT[i].SatPos, temp);
		if (success)
		{
			// �������result���Ƶ�SatPos
			for (int j = 0; j < 3; j++)
			{
				Epk->SatPVT[i].SatPos[j] = temp[j];
			}
		}
		MatrixMultiply(3, 3, 3, 1, Mat, Epk->SatPVT[i].SatVel, temp);
		for (int j = 0; j < 3; j++)
		{
			Epk->SatPVT[i].SatVel[j] = temp[j];
		}

		double Blh[3] = { 0 };
		if (Epk->SatObs[i].System == GPS)
		{
			XYZToBLH(RcvPos, Blh, R_WGS84, F_WGS84);
		}
		if (Epk->SatObs[i].System == BDS)
		{
			XYZToBLH(RcvPos, Blh, R_CGS2K, F_CGS2K);
		}

		CompSatElAz(RcvPos, Blh,Epk->SatPVT[i].SatPos, &Epk->SatPVT[i].Elevation, &Epk->SatPVT[i].Azimuth);
		//if (Epk->SatPVT[i].Elevation < PAI / 18.0)  Epk->SatPVT[i].Valid = 0;  //�߶Ƚ�����

		Epk->SatPVT[i].TropCorr = hopfield(Blh[2], Epk->SatPVT[i].Elevation);

		if (Epk->SatObs[i].System == BDS)
		{
			Epk->SatPVT[i].Tgd1= (FG1_BDS * FG1_BDS* BDSEph[prn - 1].TGD1) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
		}

	}
}
/****************************************************************************
  hopfield

  Ŀ�ģ����������ģ��

  ��ţ�405

  ����:
  H			��վ�߶�
  elev		���Ǹ߶Ƚǣ����ȣ�

  ����ֵ�����������ֵ�������վ�߶Ȳ��ڶ����㷶Χ��ֱ�����Ϊ0��
****************************************************************************/
double hopfield(const double H, const double elev)
{
	if (H >= 0 && H < 20000)
	{
		double E = Deg * elev;
		double h0 = 0;
		double t0 = 15 + 273.16;
		double p0 = 1013.25;
		double RH0 = 0.5;
		double RH = RH0 * exp(-0.0006396 * (H - h0));
		double p = p0 * pow((1 - 0.0000226 * (H - h0)), 5.225);
		double T = t0 - 0.0065 * (H - h0);
		double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
		double hw = 11000;
		double hd = 40136 + 148.72 * (t0 - 273.16);
		double kw = 155.2 * 1e-7 * (4810 / T / T) * e * (hw - H);
		double kd = 155.2 * 1e-7 * p / T * (hd - H);
		double trop = (kd / (sin(sqrt(E * E + 6.25) * Rad))) + (kw / (sin(sqrt(E * E + 2.25) * Rad)));

		return trop;
	}
	else
		return 0;
	
}