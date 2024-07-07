// SPP.cpp : ���ļ������ֲ�̽��ͽ��ջ�λ�ú��ٶȼ��㡣
//
#include "RTK_Structs.h"
// �������̽��ֲ�
//void DetectOutlier(EPOCHOBS* Obs)
//{
//	int satnum = Obs->SatNum;
//	//����һ�����ֵ��ʱ����
//	MWGF coobs[MAXCHANNUM];
//	copyArray(Obs->ComObs, coobs, MAXCHANNUM);
//	for (int i = 0; i < satnum; i++)
//	{
//		GNSSSys sys = Obs->SatObs[i].System;
//		int prn = Obs->SatObs[i].Prn;
//		double GF,dGF = 0;
//		double MW,dMW = 0;
//		double PIF = 0;
//
//		//�������ǵ�˫Ƶα�����λ�����Ƿ���Ч������������ȫ��Ϊ0����Valid���Ϊfalse��continue
//		if (Obs->SatObs[i].P[0] == 0 || Obs->SatObs[i].P[1] == 0 || Obs->SatObs[i].L[0] == 0 || Obs->SatObs[i].L[1] == 0)
//		{
//			Obs->SatObs[i].Valid = false;
//			continue;
//		}
//		//���㵱ǰ��Ԫ�����ǵ�GF��MW��PIF���ֵ
//		if (sys == GPS)
//		{
//			GF = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];
//			MW = (FG1_GPS * Obs->SatObs[i].L[0] - FG2_GPS * Obs->SatObs[i].L[1]) / (FG1_GPS - FG2_GPS) - (FG1_GPS * Obs->SatObs[i].P[0] + FG2_GPS * Obs->SatObs[i].P[1]) / (FG1_GPS + FG2_GPS);
//			PIF = (FG1_GPS * FG1_GPS * Obs->SatObs[i].P[0] - FG2_GPS * FG2_GPS * Obs->SatObs[i].P[1]) / (FG1_GPS * FG1_GPS - FG2_GPS * FG2_GPS);
//		}
//		else
//		{
//			GF = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];
//			MW = (FG1_BDS * Obs->SatObs[i].L[0] - FG3_BDS * Obs->SatObs[i].L[1]) / (FG1_BDS - FG3_BDS) - (FG1_BDS * Obs->SatObs[i].P[0] + FG3_BDS * Obs->SatObs[i].P[1]) / (FG1_BDS + FG3_BDS);
//			PIF = (FG1_BDS * FG1_BDS * Obs->SatObs[i].P[0] - FG3_BDS * FG3_BDS * Obs->SatObs[i].P[1]) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
//		}
//		//���ϸ���Ԫ��MWGF�����в��Ҹ����ǵ�GF��MW���ֵ
//		int find = 0;
//		int index = 0;
//		for (int i = 0; i < MAXCHANNUM; i++)
//		{
//			if (sys == coobs[i].Sys && prn == coobs[i].Prn)
//			{
//				find = 1;
//				index = i;
//				break;
//			}
//		}
//		//���㵱ǰ��Ԫ������GF����һ��Ԫ��ӦGF�Ĳ�ֵdGF�Ͷ�ӦMWƽ��ֵ�Ĳ�ֵdMW
//		//����Ҳ�����ֱ�Ӹ�ֵ��������޾ͱ���ԭ��Combos�����û�г��޾͸���Combos
//		if (find)
//		{
//			dGF = GF - coobs[index].GF;
//			dMW = MW - coobs[index].MW;
//			//���dGF��dMW�Ƿ��ޣ�������ޱ���ԭ��Combos
//			if (fabs(dGF) > 0.05 || fabs(dMW) > 3)
//			{
//				Obs->SatObs[i].Valid = false;
//				/*printf("slip detected sat:%d\n", Obs->SatObs[i].Prn);*/
//			}
//			else
//			{
//				//û�г��ޣ�����Combos
//				Obs->SatObs[i].Valid = true;
//				Obs->ComObs[i].Sys = sys;
//				Obs->ComObs[i].Prn = prn;
//				Obs->ComObs[i].GF = GF;
//				Obs->ComObs[i].MW = (Obs->ComObs[i].n * coobs[index].MW + MW) / (Obs->ComObs[i].n + 1);
//				Obs->ComObs[i].PIF = PIF;
//				Obs->ComObs[i].n += 1;
//			}
//		}
//		else
//		{
//			//��ǰһʱ���Ҳ������������¹۲⵽�����ǣ�ֱ��continue����
//			Obs->SatObs[i].Valid = false;
//			Obs->ComObs[i].Sys = sys;
//			Obs->ComObs[i].Prn = prn;
//			Obs->ComObs[i].GF = GF;
//			Obs->ComObs[i].MW = MW;
//			Obs->ComObs[i].PIF = PIF;
//			Obs->ComObs[i].n = 1;
//		}
//	}
//}

/*
****************************************************************************
���������ֲ�̽�⺯��
������	obs �۲�����
�������ܣ���obs�ֲ�̽�Ⲣ��SatObs����Valid,����ÿ�����ǵģ�P��IF��Ϲ۲�ֵ
****************************************************************************
*/
void DetectOutlier(EPOCHOBS* obs)
{
	// Ϊ�˲��ƻ���һ��Ԫ����Ϲ۲�ֵ����������Ļ�����
	MWGF comobs[MAXCHANNUM];// ��ŵ�ǰ��Ԫ�������ֵ
	for (int i = 0; i < obs->SatNum; i++)
	{
		// ���ÿһ�����ǵ�˫Ƶα�����λ�����Ƿ�����
		if (fabs(obs->SatObs[i].P[0]) < 1e-8 || fabs(obs->SatObs[i].P[1]) < 1e-8 || fabs(obs->SatObs[i].L[0]) < 1e-8 || fabs(obs->SatObs[i].L[1]) < 1e-8)
		{
			obs->SatObs[i].Valid = false;
			continue; //�����������ݲ�ȱ����������Ϲ۲�ֵ�ҹ۲�ֵ��Ч������Ϊfalse
		}
		/*
		// ���ý�ֹ�����
		if (fabs(obs->SatObs[i].cn0[0]) < 40 || fabs(obs->SatObs[i].cn0[1]) < 40)
		{
			obs->SatObs[i].Valid = false;
			continue; //������������Ƚϵͣ���������Ϲ۲�ֵ�ҹ۲�ֵ��Ч������Ϊfalse
		}
		*/
		// ���㵱ǰ��Ԫ�ĸ����ǵ�GF��MW���ֵ
		comobs[i].Sys = obs->SatObs[i].System;
		comobs[i].Prn = obs->SatObs[i].Prn;
		comobs[i].GF = obs->SatObs[i].L[0] - obs->SatObs[i].L[1];
		if (comobs[i].Sys == GPS)
		{
			comobs[i].MW = (FG1_GPS * obs->SatObs[i].L[0] - FG2_GPS * obs->SatObs[i].L[1]) / (FG1_GPS - FG2_GPS) -
				(FG1_GPS * obs->SatObs[i].P[0] + FG2_GPS * obs->SatObs[i].P[1]) / (FG1_GPS + FG2_GPS);
			comobs[i].PIF = (FG1_GPS * FG1_GPS * obs->SatObs[i].P[0] - FG2_GPS * FG2_GPS * obs->SatObs[i].P[1]) / (FG1_GPS * FG1_GPS - FG2_GPS * FG2_GPS);
		}
		else if (comobs[i].Sys == BDS)
		{
			comobs[i].MW = (FG1_BDS * obs->SatObs[i].L[0] - FG3_BDS * obs->SatObs[i].L[1]) / (FG1_BDS - FG3_BDS) -
				(FG1_BDS * obs->SatObs[i].P[0] + FG3_BDS * obs->SatObs[i].P[1]) / (FG1_BDS + FG3_BDS);
			comobs[i].PIF = (FG1_BDS * FG1_BDS * obs->SatObs[i].P[0] - FG3_BDS * FG3_BDS * obs->SatObs[i].P[1]) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
		}

		comobs[i].n = 1;

		// ���ϸ���Ԫ��MWGF�����в��Ҹ����ǵ�GF��MW���ֵ
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (obs->ComObs[j].Sys == comobs[i].Sys && obs->ComObs[j].Prn == comobs[i].Prn)
			{
				// �ҵ�����Ƚ϶����Ƿ����޲���
				double dGF = fabs(comobs[i].GF - obs->ComObs[j].GF);
				double dMW = fabs(comobs[i].MW - obs->ComObs[j].MW);

				// û�г��ޣ�����true������MW��ƽ��ֵ
				if (dGF < 0.05 && dMW < 3)
				{
					obs->SatObs[i].Valid = true;
					comobs[i].MW = (obs->ComObs[j].MW * obs->ComObs[j].n + comobs[i].MW) / (obs->ComObs[j].n + 1);
					comobs[i].n = obs->ComObs[j].n + 1;
				}
				break;
				// ���ޣ�����Ϊ�ֲ����ʼ��������false���ʲ��账��
			}
			// ������ϸ���Ԫ��û���ҵ�����֪���Ƿ���ã�Ĭ��Ϊ��ʼ��ʱ��false
			else continue;
		}
	}
	// ����������Ϲ۲�ֵ�Ŀ�����obs��
	memcpy(obs->ComObs, comobs, sizeof(comobs));
}
/****************************************************************************
  SPP

  Ŀ�ģ����㶨λ�����

  ��ţ�501

  ����:
  Epoch			�۲����ݽṹ��
  GpsEph		GPS��������
  BDSEph		BDS��������
  Res		    �û���λ���

  ����ֵ������ɹ�����true������4�����ǻ��߾�������ʧ�ܷ���false
****************************************************************************/
bool SPP(EPOCHOBS* Epoch, RAWDAT* Raw, PPRESULT* Result)
{
	//�������Ҫ��������
	double RcvPos[3] = { 0 };//�״ζ�λ����Ϊ0
	RcvPos[0] = Result->Position[0];
	RcvPos[1] = Result->Position[1];
	RcvPos[2] = Result->Position[2];
	double dt[2] = { 0,0 };//ÿ�ε������ʱ�䣬m
	int validsat = 0;//��Ч�۲�������
	int gpscount = 0;//��Ч�۲�GPS������
	int bdscount = 0;//��Ч�۲�BDS������
	int parnum = 5;/*��Ч��������*/
	double pdop = 0;//PDOPֵ���ռ�λ�þ�������
	double sigma0 = 0;//���λȨ�����

	int Iterator = 0;  /*�������������С��10��*/
	double dRcv[3] = { 0 };//ÿ�ε�����λ�ñ仯��
	//�ֲ�̽��
	DetectOutlier(Epoch);
	do {

		ComputeGPSSatOrbitAtSignalTrans(Epoch, Raw->GpsEph, Raw->BdsEph, RcvPos);
		validsat = 0;//��Ч�۲�������
		gpscount = 0;//��Ч�۲�GPS������
		bdscount = 0;//��Ч�۲�BDS������
		//��ʼ���۲����
		double B[MAXCHANNUM * 5] = { 0 };
		double w[MAXCHANNUM * 1] = { 0 };
		for (int i = 0; i < Epoch->SatNum; i++)
		{
			//����λ�ü���ʧ�ܡ��۲����ݲ��������дֲ�����붨λ����
			if ((!Epoch->SatPVT[i].Valid) || (!Epoch->SatObs[i].Valid)) continue;
			else validsat++;

			double dPos[3] = { 0 };//ÿ���۲����ǵ����ջ��ľ���
			MatrixSubtraction(3, 1, Epoch->SatPVT[i].SatPos, RcvPos, dPos);

			//�۲ⷽ�����Ի�,����B��w����
			double d = Norm(3, dPos);
			double l = (RcvPos[0] - Epoch->SatPVT[i].SatPos[0])/d;
			double m = (RcvPos[1] - Epoch->SatPVT[i].SatPos[1]) / d;
			double n = (RcvPos[2] - Epoch->SatPVT[i].SatPos[2]) / d;
			double w0 = 0;
			AdjRow(MAXCHANNUM, 5, 0, validsat, l, B);
			AdjRow(MAXCHANNUM, 5, 1, validsat, m, B);
			AdjRow(MAXCHANNUM, 5, 2, validsat, n, B);
			if (Epoch->SatObs[i].System == GPS)
			{
				AdjRow(MAXCHANNUM, 5, 3, validsat, 1, B);
				AdjRow(MAXCHANNUM, 5, 4, validsat, 0, B);
				w0 = Epoch->ComObs[i].PIF - (d + dt[0] - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr + C_Light * Epoch->SatPVT[i].Tgd1);
				AdjRow(MAXCHANNUM, 1, 0, validsat, w0, w);
				gpscount++;
			}
			if (Epoch->SatObs[i].System == BDS)
			{
				AdjRow(MAXCHANNUM, 5, 3, validsat, 0, B);
				AdjRow(MAXCHANNUM, 5, 4, validsat, 1, B);
				w0 = Epoch->ComObs[i].PIF - (d + dt[1] - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr + C_Light * Epoch->SatPVT[i].Tgd1);
				AdjRow(MAXCHANNUM, 1, 0, validsat, w0, w);
				bdscount++;
			}
		}
		//����4�����ǣ������ж�λ����
		if (validsat < 4)
		{
			Result->IsSuccess = false;
			return false;
		}
		//ȷ����������
		if (gpscount == 0|| bdscount == 0)
		{
			parnum = 4;
		}

		double P[MAXCHANNUM * 1] = { 0 };
		for (int i = 0; i < MAXCHANNUM; i++)
		{
			P[i] = 1;
		}
		double BT[5 * MAXCHANNUM] = { 0 };
		double NBB0[5 * 5] = { 0 };

		double* Qxx = new double[parnum * parnum]();//NBB����
		double* W = new double[parnum * 1]();
		double* x = new double[parnum]();//��λm
		double* B0 = new double[MAXCHANNUM*parnum]();

		double Bx[MAXCHANNUM*1] = { 0 };
		double v[MAXCHANNUM * 1] = { 0 };
		double vT[MAXCHANNUM * 1] = { 0 };
		double sigma = 0;

		//��С�������
		MatrixTranspose(MAXCHANNUM, 5, B, BT);
		MatrixMultiply_APB(5, MAXCHANNUM, MAXCHANNUM, 5, BT, P, B, NBB0);
		double* NBB = new double[parnum * parnum]();

		double W0[5 * 1] = { 0 };
		MatrixMultiply_APB(5, MAXCHANNUM, MAXCHANNUM, 1, BT, P, w, W0);
		//�۲����ݴ��������������Ҫ��Nxx0�����W0�Լ�B�ع�
		if (gpscount == 0)
		{
			compressMatrix(4, 4, NBB0, 5, 5, NBB);
			compressMatrix(4, -1, W0, 5, 1, W);
			compressMatrix(-1, 4, B, MAXCHANNUM, 5, B0);
		}
		else if (bdscount == 0)
		{
			compressMatrix(5, 5, NBB0, 5, 5, NBB);
			compressMatrix(5, -1, W0, 5, 1, W);
			compressMatrix(-1, 5, B, MAXCHANNUM, 5, B0);
		}
		else
		{
			compressMatrix(-1, -1, NBB0, 5, 5, NBB);
			compressMatrix(-1, -1, W0, 5, 1, W);
			compressMatrix(-1, -1, B, MAXCHANNUM, 5, B0);
		}
		if (!MatrixInv(parnum, NBB, Qxx))
		{
			Result->IsSuccess = false;
			return false;
		}
		MatrixMultiply(parnum, parnum, parnum, 1, Qxx, W, x);
		//��λ��������
		MatrixMultiply(MAXCHANNUM, parnum, parnum, 1, B0, x, Bx);
		MatrixSubtraction(MAXCHANNUM, 1, Bx, w, v);
		MatrixTranspose(MAXCHANNUM, 1, v, vT);
		MatrixMultiply_APB(1, MAXCHANNUM, MAXCHANNUM, 1, vT, P, v, &sigma);
		sigma0 = sqrt(sigma / (validsat - parnum));
		pdop = sqrt(Qxx[0] + Qxx[parnum + 1] + Qxx[2 * parnum + 2]);
		//�ı���ջ�λ�ã�Ϊ��һ��ѭ����׼��
		dRcv[0] = x[0];
		dRcv[1] = x[1];
		dRcv[2] = x[2];
		if (bdscount == 0)
		{
			dt[0] += x[3];
		}
		else if(gpscount == 0)
		{
			dt[1] += x[3];
		}
		else
		{
			dt[0] += x[3];
			dt[1] += x[4];
		}
		MatrixAddition2(3, 1, dRcv, RcvPos);
		Iterator++;
		//��̬�����ͷ�
		delete []Qxx ;
		delete []W ; 
		delete []x ;
		delete []B0 ; 
		delete []NBB;
	}while(Norm(3, dRcv)> 1e-3 && Iterator < 10);
	Result->Time = Epoch->Time;
	Result->Position[0] = RcvPos[0];
	Result->Position[1] = RcvPos[1];
	Result->Position[2] = RcvPos[2];
	Result->RcvClkOft[0] = dt[0];
	Result->RcvClkOft[1] = dt[1];
	Result->PDOP = pdop;
	Result->SigmaPos = sigma0;
	Result->GPSSatNum = gpscount;
	Result->BDSSatNum = bdscount;
	Result->AllSatNum = validsat;
	Result->AllSatNum = validsat;
	Result->IsSuccess = true;
	
	SPV(Epoch, Result);//�������


	XYZToBLH(RcvPos, Result->BLH, R_CGS2K, F_CGS2K);
	Comp_dEnu(Epoch->Pos, Result->Position, Result->dENH);
	return true;
}

/****************************************************************************
  SPV

  Ŀ�ģ��������

  ��ţ�503

  ����:
  Epoch			�۲����ݽṹ��
  Res		    �û���λ���

  ����ֵ������ɹ���ʧ��
****************************************************************************/
void SPV(EPOCHOBS* Epoch, PPRESULT* Result)
{
	int validsat = 0;//��Ч������
	double RcvPos[3] = { 0 };
	RcvPos[0] = Result->Position[0];
	RcvPos[1] = Result->Position[1];
	RcvPos[2] = Result->Position[2];

	double sigma0 = 0;
	double x[4 * 1] = { 0 };//��λm
	//��ʼ���۲����
	double B[MAXCHANNUM * 4] = { 0 };
	double w[MAXCHANNUM * 1] = { 0 };
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if ((!Epoch->SatPVT[i].Valid) || (!Epoch->SatObs[i].Valid)) continue;
		else validsat++;

		double dPos[3] = { 0 };//ÿ���۲����ǵ����ջ��ľ���
		MatrixSubtraction(3, 1, Epoch->SatPVT[i].SatPos, RcvPos, dPos);

		//�۲ⷽ�����Ի�,����B��w����
		double d = Norm(3, dPos);
		double l = (RcvPos[0] - Epoch->SatPVT[i].SatPos[0]) / d;
		double m = (RcvPos[1] - Epoch->SatPVT[i].SatPos[1]) / d;
		double n = (RcvPos[2] - Epoch->SatPVT[i].SatPos[2]) / d;
		double w0 = 0;
		double dd = ((Epoch->SatPVT[i].SatPos[0] - RcvPos[0]) * Epoch->SatPVT[i].SatVel[0] + (Epoch->SatPVT[i].SatPos[1] - RcvPos[1]) * Epoch->SatPVT[i].SatVel[1] + (Epoch->SatPVT[i].SatPos[2] - RcvPos[2]) * Epoch->SatPVT[i].SatVel[2]) / d;
		AdjRow(MAXCHANNUM, 4, 0, validsat, l, B);
		AdjRow(MAXCHANNUM, 4, 1, validsat, m, B);
		AdjRow(MAXCHANNUM, 4, 2, validsat, n, B);
		AdjRow(MAXCHANNUM, 4, 3, validsat, 1, B);
		w0 = Epoch->SatObs[i].D[0] - (dd -C_Light * Epoch->SatPVT[i].SatClkSft);
		AdjRow(MAXCHANNUM, 1, 0, validsat, w0, w);
	}
	double P[MAXCHANNUM * 1] = { 0 };
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		P[i] = 1;
	}
	double BT[4 * MAXCHANNUM] = { 0 };
	double Nxx0[4 * 4] = { 0 };
	double Nxx[4 * 4] = { 0 };
	double W[4 * 1] = { 0 };

	double Bx[MAXCHANNUM * 1] = { 0 };
	double v[MAXCHANNUM * 1] = { 0 };
	double vT[MAXCHANNUM * 1] = { 0 };
	double sigma = 0;

	//��С������Ⲣ��������
	MatrixTranspose(MAXCHANNUM, 4, B, BT);
	MatrixMultiply_APB(4, MAXCHANNUM, MAXCHANNUM, 4, BT, P, B, Nxx0);
	MatrixInv(4, Nxx0, Nxx);
	MatrixMultiply_APB(4, MAXCHANNUM, MAXCHANNUM, 1, BT, P, w, W);
	MatrixMultiply(4, 4, 4, 1, Nxx, W, x);
	MatrixMultiply(MAXCHANNUM, 4, 4, 1, B, x, Bx);
	MatrixSubtraction(MAXCHANNUM, 1, Bx, w, v);
	MatrixTranspose(MAXCHANNUM, 1, v, vT);
	MatrixMultiply_APB(1, MAXCHANNUM, MAXCHANNUM, 1, vT, P, v, &sigma);
	sigma0 = sqrt(sigma / (validsat - 4));

	//��ֵ����λ����ṹ��
	Result->Velocity[0] = x[0];
	Result->Velocity[1] = x[1];
	Result->Velocity[2] = x[2];
	Result->RcvClkSft = x[3];
	Result->SigmaVel = sigma0;
}