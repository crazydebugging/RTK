// RTKEKF.cpp : ���ļ�����RTK�������˲����㡣
//
#include "RTK_Structs.h"
/****************************************************************************
  InitEKF

  Ŀ�ģ��˲���ʼ��(����������״̬Э����)


  ��ţ�701

  ����:

****************************************************************************/
bool InitEKF(DDCOBS* DDObs, double* RovPos,int& parnum, double* X, double* P)
{
	int sys = 0;
	//λ�ø���ֵ
	for (int i = 0; i < 3; i++)
	{
		X[i] = RovPos[i];
		P[i * parnum + i] = 10;
	}
	//N����ֵ
	int i = 3;//ʵ������
	for (int i0 = 0; i0 < DDObs->Sats; i0++)
	{
		if (DDObs->ddSatObs[i0].System == GPS) sys = 0;
		else if (DDObs->ddSatObs[i0].System == BDS) sys = 1;
		double WL1 = 0, WL2 = 0;
		if (sys == 0)
		{
			WL1 = WL1_GPS;  WL2 = WL2_GPS;
		}
		else
		{
			WL1 = WL1_BDS;  WL2 = WL3_BDS;
		}
		X[i] = (DDObs->ddSatObs[i0].ddL[0] - DDObs->ddSatObs[i0].ddP[0]) / WL1;
		X[i + 1] = (DDObs->ddSatObs[i0].ddL[1] - DDObs->ddSatObs[i0].ddP[1]) / WL2;
		P[i * parnum + i] = 6 / WL1;
		P[(i + 1) * parnum + i + 1] = 6 / WL2;
		i += 2;
	}
	if (i == parnum) return true;
	else return false;
}
/****************************************************************************
  Prediction

  Ŀ�ģ�ʱ��Ԥ��


  ��ţ�702

  ����:

****************************************************************************/
void Prediction(DDCOBS* DDObs, DDCOBS* CurDDObs, int* Index,int& Curparnum, int& parnum,double*X0, double* X1,double*P0,double*P1)
{
	double* phi = new double[Curparnum * parnum]();
	double* Q = new double[Curparnum * Curparnum]();
	//��������
	int index[MAXCHANNUM] = { 0 };
	for (int i = 0; i < DDObs->Sats; i++)
	{
		index[i] = Index[i];
	}
	for (int i = 0; i < MAXCHANNUM; i++) Index[i]= 0;
	//λ�ô���
	for (int i = 0; i < 3; i++)
	{
		phi[i* parnum + i] = 1;
	}
	//�ο��Ǹı�ʱ
	int find = 0;
	int prnindex = 0;
	//ģ���ȴ���,���ݳɹ��������µ�IndexΪ1��ʧ��Ϊ0
	for (int i = 0; i < CurDDObs->Sats; i++)
	{
		int Cursys = 0;
		int Curprn = 0;
		if (CurDDObs->ddSatObs[i].System == GPS) Cursys = 0;
		else if (CurDDObs->ddSatObs[i].System == BDS) Cursys = 1;
		Curprn = CurDDObs->ddSatObs[i].Prn;

		int j = 0;
		int sys = 0;
		int prn = 0;
		for (j = 0; j < DDObs->Sats; j++)
		{
			if (index[j] == 1)
			{
				if (DDObs->ddSatObs[j].System == GPS) sys = 0;
				else if (DDObs->ddSatObs[j].System == BDS) sys = 1;
				prn = DDObs->ddSatObs[j].Prn;
				//�ο���δ�ı�ʱ
				if (sys == Cursys && CurDDObs->RefPrn[Cursys] == DDObs->RefPrn[sys])
				{
					if (prn == Curprn)
					{
						phi[(3 + 2 * i) * parnum + (3 + j * 2)] = 1; phi[(3 + 2 * i + 1) * parnum + (3 + j * 2 + 1)] = 1;
						Index[i] = 1;
						break;
					}
					else Index[i] = 0;
				}
				//�ο��Ǹı�ʱ
				else if (Cursys == sys && CurDDObs->RefPrn[Cursys] != DDObs->RefPrn[sys])
				{
					//��ÿһ�ŵ�ǰ��Ԫ���ǣ������ϸ���Ԫ����
					if (sys == 0 && CurDDObs->RefPrn[Cursys] == prn)
					{
						prnindex = j;
						find = 1;
						break;
					}
					else if (sys == 1 && CurDDObs->RefPrn[Cursys] == prn)
					{
						prnindex = j;
						find = 2;
						break;
					}
					else find = 0;
				}
			}
		}
		for (j = 0; j < DDObs->Sats; j++)
		{
			//�ο��Ǹı䲢�ҵ�ʱ,������ǰ����
			if (find>=1)
			{
				if (index[j] == 1)
				{
					if (DDObs->ddSatObs[j].System == GPS) sys = 0;
					else if (DDObs->ddSatObs[j].System == BDS) sys = 1;
					prn = DDObs->ddSatObs[j].Prn;
					if (sys == Cursys && prn == Curprn)
					{
						phi[(3 + 2 * i) * parnum + (3 + j * 2)] = 1; phi[(3 + 2 * i + 1) * parnum + (3 + j * 2 + 1)] = 1;
						phi[(3 + 2 * i) * parnum + (3 + prnindex * 2)] = -1; phi[(3 + 2 * i + 1) * parnum + (3 + prnindex * 2 + 1)] = -1;
						Index[i] = 1;
						break;
					}
					else if (sys == Cursys && DDObs->RefPrn[Cursys] == prn)
					{
						phi[(3 + 2 * i) * parnum + (3 + j * 2)] = -1; phi[(3 + 2 * i + 1) * parnum + (3 + j * 2 + 1)] = -1;
						Index[i] = 1;
						break;
					}
					else Index[i] = 0;
				}
			}
			//�ο��Ǹı䵫û�ҵ�����ǰ��Ԫ����ȫ���Ҳ�����������ǲο���δ�ı䣬��������ж�����
			else if(Index[i]!=1) Index[i] = 0;
		}
	}
	MatrixMultiply(Curparnum, parnum, parnum, 1, phi, X0, X1);
	//��Q��ʼ��
	for (int i = 0; i <Curparnum; i++)
	{
		if(i<3)	Q[i * Curparnum + i] = 1e-8;
		else Q[i * Curparnum + i] = 1e-10;
	}
	MatrixMultiply_ABAT_C(Curparnum, parnum, parnum, Curparnum, phi, P0, Q, P1);
	//����̽��,��˫��۲�ֵ��������һ��Ԫ���ݵ�N̽��ֲ��������������ֵ���б�ǣ�������������ֵ���Ϊ-1
	DDetectCycleSlip(CurDDObs);
	/*DetectHalf(DDObs, CurDDObs);*/
	for (int i = 0; i < CurDDObs->Sats; i++)
	{
		if (Index[i] == 1)
		{
			int Cursys = 0;
			if (CurDDObs->ddSatObs[i].System == GPS) Cursys = 0;
			else if (DDObs->ddSatObs[i].System == BDS) Cursys = 1;
			double WL1 = 0, WL2 = 0, N1 = 0, N2 = 0;
			if (Cursys == 0)
			{
				WL1 = WL1_GPS;  WL2 = WL2_GPS;
			}
			else
			{
				WL1 = WL1_BDS;  WL2 = WL3_BDS;
			}
			N1 = (DDObs->ddSatObs[i].ddL[0] - DDObs->ddSatObs[i].ddP[0]) / WL1;
			N2 = (DDObs->ddSatObs[i].ddL[1] - DDObs->ddSatObs[i].ddP[1]) / WL2;
			if ((fabs(N1 - X1[3 + i * 2]) > 30) || (fabs(N2 - X1[i * 2 + 4]) > 30)) CurDDObs->ddSatObs[i].Valid = false;
		}
		if (CurDDObs->ddSatObs[i].Valid == false) Index[i] = -1;
	}
	//��Xkk1��δ�ɹ����ݵĹ̶�ģ���Ƚ��г�ʼ����һ������Ϊ�����Ԫ�����������ǻ��߳��ֲִ״̬��״̬Э������
	for (int i = 0; i < CurDDObs->Sats; i++)
	{
		if (Index[i] != 1)
		{
			int Cursys = 0;
			if (CurDDObs->ddSatObs[i].System == GPS) Cursys = 0;
			else if (CurDDObs->ddSatObs[i].System == BDS) Cursys = 1;
			double WL1 = 0, WL2 = 0;
			if (Cursys == 0)
			{
				WL1 = WL1_GPS;  WL2 = WL2_GPS;
			}
			else
			{
				WL1 = WL1_BDS;  WL2 = WL3_BDS;
			}
			X1[3+i*2] = (CurDDObs->ddSatObs[i].ddL[0] - CurDDObs->ddSatObs[i].ddP[0]) / WL1;
			X1[i*2 + 4] = (CurDDObs->ddSatObs[i].ddL[1] - CurDDObs->ddSatObs[i].ddP[1]) / WL2;
			P1[(3 + i * 2)*Curparnum+ (3 + i * 2)]= 6 / WL1;
			P1[(4 + i * 2) * Curparnum + (4 + i * 2)] = 6 / WL2;
		}
	}
	delete []phi; delete[]Q;
}
/****************************************************************************
  Update

  Ŀ�ģ���������,����ʹ���������Ĺ۲⼰���Ӧ��ģ����

  ��ţ�703

  ����:

****************************************************************************/
//void Update(SDEPOCHOBS* CurSDObs, DDCOBS* CurDDObs, double* BasPos, EPOCHOBS* BasEpk, EPOCHOBS* RovEpk,int* Index,double* X0, double* X1, double* P0, double* P1)
//{
//
//}
/****************************************************************************
  DDUpdate

  Ŀ�ģ�����˫��۲�ֵ��������,����ʹ���������Ĺ۲⼰���Ӧ��ģ����

  ��ţ�703

  ����:

****************************************************************************/
bool DDUpdate(SDEPOCHOBS* CurSDObs, DDCOBS* CurDDObs, double* BasPos,EPOCHOBS*BasEpk, EPOCHOBS* RovEpk, int* Index, double* X1,double* P1)
{
	//����GPS��BDS˫��������(������)
	int validsat = CurDDObs->Sats;//�۲�˫��������
	double gpscount = CurDDObs->DDSatNum[0];//�۲�GPS˫��������
	double bdscount = CurDDObs->DDSatNum[1];//�۲�BDS˫��������
	//ȷ����������
	int parnum = 3 + 2 * validsat;
	double sigma0 = 0;//���λȨ�����
	double pdop = 0;//PDOPֵ���ռ�λ�þ�������
	//ʹ��ʱ��Ԥ�������е���
	double RovPos[3] = { 0 };
	double* N = new double[validsat * 2]();//˫����ģ���ȣ���
	for (int i = 0; i < parnum; i++)
	{
		if(i<3)RovPos[i] = X1[i];
		else N[i - 3] = X1[i];
	}
	//�����վ���굽���е������ǵļ��ξ���
	double BasSat[MAXCHANNUM] = { 0 };
	for (int i = 0; i < CurSDObs->SatNum; i++)
	{
		int BasIndex = CurSDObs->SdSatObs[i].nBas;
		if (CurSDObs->SdSatObs[i].Valid == true)//����������Ч������λ�ü���ɹ�
		{
			double dPos[3] = { 0 };//ÿ���۲����ǵ���վ�ľ���
			MatrixSubtraction(3, 1, BasEpk->SatPVT[BasIndex].SatPos, BasPos, dPos);
			BasSat[i] = Norm(3, dPos);
		}
	}
	//��������վ���ο��ǵļ��ξ���
	double RovRefSat[2] = { 0 };
	for (int i = 0; i < 2; i++)
	{
		if (CurDDObs->RefPos[i] >= 0)//�ο���ѡȡ�ɹ�
		{
			int RovIndex = CurSDObs->SdSatObs[CurDDObs->RefPos[i]].nRov;
			double dPos[3] = { 0 };//ÿ���۲����ǵ���վ�ľ���
			MatrixSubtraction(3, 1, RovEpk->SatPVT[RovIndex].SatPos, RovPos, dPos);
			RovRefSat[i] = Norm(3, dPos);
		}
	}
	//����Ȩ����;���Ի�˫��۲ⷽ�̵õ�B�����W����,�ο��ǲ��ü���
	double* B = new double[(validsat * 4) * parnum]();
	double* w = new double[(validsat * 4)]();
	double* R = new double[(validsat * 4) * (validsat * 4)]();
	int i = 0;//ʵ�ʾ���������
	int sys = 0;
	for (int i0 = 0; i0 < CurDDObs->Sats; i0++)//��˫��۲�ֵ����ѭ��
	{
		if (CurDDObs->ddSatObs[i0].System == GPS) sys = 0;
		else if (CurDDObs->ddSatObs[i0].System == BDS) sys = 1;

		if (CurDDObs->ddSatObs[i0].Valid != true) continue;
		int di = CurDDObs->ddSatObs[i0].Pos;//��ǰ˫�������ڵ��������е�λ��
		int RovIndex = CurSDObs->SdSatObs[di].nRov;//��ǰ��������������վ�۲��е�λ��
		int RovRefIndex = CurDDObs->RefPos[sys];//�ο������ڵ����е����
		double dPos[3] = { 0 };//ÿ��˫��۲����ǵ�����վ�ľ���
		MatrixSubtraction(3, 1, RovEpk->SatPVT[RovIndex].SatPos, RovPos, dPos);
		double d = Norm(3, dPos);
		double l = (RovPos[0] - RovEpk->SatPVT[RovIndex].SatPos[0]) / d - (RovPos[0] - RovEpk->SatPVT[CurSDObs->SdSatObs[RovRefIndex].nRov].SatPos[0]) / RovRefSat[sys];
		double m = (RovPos[1] - RovEpk->SatPVT[RovIndex].SatPos[1]) / d - (RovPos[1] - RovEpk->SatPVT[CurSDObs->SdSatObs[RovRefIndex].nRov].SatPos[1]) / RovRefSat[sys];
		double n = (RovPos[2] - RovEpk->SatPVT[RovIndex].SatPos[2]) / d - (RovPos[2] - RovEpk->SatPVT[CurSDObs->SdSatObs[RovRefIndex].nRov].SatPos[2]) / RovRefSat[sys];
		double rou = d - RovRefSat[sys] - BasSat[di] + BasSat[RovRefIndex];
		for (int j = 0; j < 4; j++)
		{
			B[(i * 4 + j) * (3 + 2 * validsat)] = l;
			B[(i * 4 + j) * (3 + 2 * validsat) + 1] = m;
			B[(i * 4 + j) * (3 + 2 * validsat) + 2] = n;
			if (j % 4 == 0 || j % 4 == 1)
			{
				w[(i * 4 + j)] = CurDDObs->ddSatObs[i0].ddP[j] - rou;
				if (sys == 0)
				{
					for (int k = 0; k < gpscount; k++)
					{
						if (k == i)
						{
							R[(i * 4 + j) * (validsat * 4 + 1)] = 2 * 9 * 2;
						}
						else R[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = 2 * 9 * 1;
					}
				}
				else
				{
					for (int k = gpscount; k < gpscount + bdscount; k++)
					{
						if (k == i)
						{
							R[(i * 4 + j) * (validsat * 4 + 1)] = 2 * 9 * 2;
						}
						else R[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = 2 * 9 * 1;
					}
				}
			}
			if (j % 4 == 2 || j % 4 == 3)
			{
				if (sys == 0)
				{
					if (j % 4 == 2)
					{
						B[(i * 4 + j) * (3 + 2 * validsat) + 2 + i * 2 + 1] = WL1_GPS;
						w[(i * 4 + j)] = CurDDObs->ddSatObs[i0].ddL[j - 2] - rou - WL1_GPS * N[i * 2];
					}
					if (j % 4 == 3)
					{
						B[(i * 4 + j) * (3 + 2 * validsat) + 2 + i * 2 + 2] = WL2_GPS;
						w[(i * 4 + j)] = CurDDObs->ddSatObs[i0].ddL[j - 2] - rou - WL2_GPS * N[i * 2 + 1];
					}
					for (int k = 0; k < gpscount; k++)
					{
						if (k == i)
						{
							R[(i * 4 + j) * (validsat * 4 + 1)] = 2 * 0.0009 * 2;
						}
						else R[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = 2 * 0.0009 * 1;
					}
				}
				else
				{
					if (j % 4 == 2 || j % 4 == 3)
					{
						if (j % 4 == 2)
						{
							B[(i * 4 + j) * (3 + 2 * validsat) + 2 + i * 2 + 1] = WL1_BDS;
							w[(i * 4 + j)] = CurDDObs->ddSatObs[i0].ddL[j - 2] - rou - WL1_BDS * N[i * 2];
						}
						if (j % 4 == 3)
						{
							B[(i * 4 + j) * (3 + 2 * validsat) + 2 + i * 2 + 2] = WL3_BDS;
							w[(i * 4 + j)] = CurDDObs->ddSatObs[i0].ddL[j - 2] - rou - WL3_BDS * N[i * 2 + 1];
						}
						for (int k = gpscount; k < gpscount + bdscount; k++)
						{
							if (k == i)
							{
								R[(i * 4 + j) * (validsat * 4 + 1)] = 2 * 0.0009 * 2;
							}
							else R[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = 2 * 0.0009 * 1;
						}
					}
				}
			}
		}
		i++;
	}
	//��ӡB�����W����,Ȩ����
		//outputMatrixToFile(B, (validsat * 4), (3 + 2 * validsat), "outputB.txt");
		//outputMatrixToFile(w, (validsat * 4), 1, "outputw.txt");
		//outputMatrixToFile(R, (validsat * 4), (validsat * 4), "outputR.txt");
	if (validsat < 4)
	{
		CurDDObs->IsSuccess = false;
		return false;
	}
	//�˲���������
	double* BPR = new double[(validsat * 4) * (validsat * 4)]();
	double* NBPR = new double[(validsat * 4) * (validsat * 4)]();
	double* BT = new double[parnum * (validsat * 4)]();
	double* K = new double[parnum*(validsat * 4)]();
	double* Kw= new double[parnum]();
	double* KB = new double[parnum * parnum]();
	double*I= new double[parnum * parnum]();
	for (int i = 0; i < parnum; i++)
	{
		I[i* parnum+ i] = 1;
	}
	double*IKB= new double[parnum * parnum]();
	double* IPI = new double[parnum * parnum]();
	double* KRK = new double[parnum * parnum]();	
	MatrixMultiply_ABAT_C((validsat * 4), parnum, parnum, (validsat * 4), B, P1, R, BPR);
	
	if (!lu_decomposition_inverse((validsat * 4), BPR, NBPR))
	{
		CurDDObs->IsSuccess = false;
		return false;
	}
	MatrixTranspose((validsat * 4), parnum, B, BT);
	MatrixMultiply(parnum, (validsat * 4), (validsat * 4), 1, K, w, Kw);
	MatrixMultiply_ABC(parnum, parnum, parnum, (validsat * 4), (validsat * 4), (validsat * 4), P1, BT, NBPR, K);
	MatrixMultiply(parnum, (validsat * 4), (validsat * 4), 1, K, w, Kw);
	MatrixAddition2(parnum, 1, Kw, X1);
	MatrixMultiply(parnum, (validsat * 4), (validsat * 4), parnum, K, B, KB);
	MatrixSubtraction(parnum, parnum, I, KB, IKB);
	MatrixMultiply_ABAT(parnum, parnum, parnum, IKB, P1, IPI);
	MatrixMultiply_ABAT(parnum, (validsat * 4), (validsat * 4), K, R, KRK);
	MatrixAddition(parnum, parnum, IPI, KRK, P1);
	
	double* v = new double[(validsat * 4)]();
	double* Bx = new double[(validsat * 4)]();
	double* P = new double[(validsat * 4) * (validsat * 4)]();
	//��λ��������
	/*MatrixMultiply((validsat * 4), parnum, parnum, 1, B, X1, Bx);
	MatrixSubtraction((validsat * 4), 1, Bx, w, v);*/
	double sigma = 0;
	if (!lu_decomposition_inverse((validsat * 4), R, P))
	{
		CurDDObs->IsSuccess = false;
		return false;
	}
	MatrixMultiply_ATPPA((validsat * 4), 1, (validsat * 4), w, P, &sigma);
	sigma0 = sqrt(sigma / (validsat * 4 - parnum));
	pdop = sqrt(P1[0] + P1[parnum + 1] + P1[2 * parnum + 2]);
	//��̬�����ͷ�
	delete[]B;delete[]w;delete[]R;delete[]BPR;delete[]NBPR;delete[]BT;
	delete[]K; delete[]Kw; delete[]KB; delete[]I; delete[]IKB; delete[]IPI; delete[]KRK;
	//����˫����ģ���ȼ���Э������������LAMBDAģ���ȹ̶�
	double* Qnn = new double[(2 * validsat) * (2 * validsat)]();//˫����ģ����Э��������
	for (int i = 0; i < 2 * validsat; i++) {
		N[i] = X1[3 + i];
		for (int j = 0; j < 2 * validsat; j++) {
			Qnn[i * (2 * validsat) + j] = P1[(parnum - 2 * validsat + i) * parnum + (parnum - 2 * validsat + j)];
		}
	}
	CurDDObs->IsSuccess = true;
	//LAMBDAģ���ȹ̶�
	if (lambda(validsat * 2, 2, N, Qnn, CurDDObs->FixedAmb, CurDDObs->ResAmb) != 0)
	{
		CurDDObs->bFixed = false;
		CurDDObs->Ratio = 0;
		for (int i = 0; i < CurDDObs->Sats; i++)
		{
			if (Index[i] != 1)Index[i] = -1;
		}
		for (int i = 0; i < 3; i++)
		{
			RovPos[i] = X1[i];
		}
		MatrixSubtraction(3, 1, RovPos, BasPos, CurDDObs->dPos);// ��������
		Comp_dEnu(BasPos, RovPos, CurDDObs->dENH);
		CurDDObs->FixRMS[0] = sigma0 * pdop;// ����ⶨλ��rms���
		return false;
	}
	else
	{
		CurDDObs->bFixed = true;
		for (int i = 0; i < 2 * validsat; i++) {
			X1[3 + i]= CurDDObs->FixedAmb[i];
		}
		CurDDObs->Ratio = CurDDObs->ResAmb[1] / CurDDObs->ResAmb[0];//Ratioֵ
		for (int i = 0; i < CurDDObs->Sats; i++)
		{
			if (Index[i] == 0) Index[i] = 1;
		}
		for (int i = 0; i < 3; i++)
		{
			RovPos[i] = X1[i];
		}
		MatrixSubtraction(3, 1, RovPos, BasPos, CurDDObs->dPos);// ��������
		Comp_dEnu(BasPos, RovPos, CurDDObs->dENH);
		CurDDObs->FixRMS[0] = sigma0 * pdop;// ����ⶨλ��rms���
	}
	delete[]N;delete[]Qnn;
	return true;
}
/****************************************************************************
  RTKekf

  Ŀ�ģ�˫����Զ�λ�����(�������˲���)


  ��ţ�704

  �������:
  Raw			RTK��λ���ݣ���ԭʼ�۲�ֵ��������λ�ü�����м�����������۲�ֵ��˫���׼��ѡȡ���������
  Rov			����վSPP��λ���

  ����ֵ�����������Ƿ�ɹ�������ɹ�����true������4�����ǻ��߾�������ʧ�ܷ���false
****************************************************************************/
bool RTKekf(RAWDAT* Raw, PPRESULT* Rov, RTKEKF*EKF)
{
	#pragma omp parallel for  // ʹ��OpenMP���в��м���
	EKF->DDObs = EKF->CurDDObs;//��ͬ��ǰ����һ��ԭʼ�۲�ֵ����
	//�۲�ֵ
	DetectLockTimeHalf(&Raw->BBasEpk, &Raw->BasEpk); DetectLockTimeHalf(&Raw->BRovEpk, &Raw->RovEpk);//���ý��ջ�����������Locktime�����������
	//����۲�ֵ����������
	FormSDEpochObs(&Raw->RovEpk, &Raw->BasEpk, &EKF->CurSDObs);//վ�䵥��۲�ֵ����
	DetectCycleSlip(&EKF->CurSDObs);//��վ�䵥��۲�ֵ����MW��GF���̽������
	DetRefSat(&Raw->RovEpk, &Raw->BasEpk, &EKF->CurSDObs, &EKF->CurDDObs);//ѡȡ��׼��
	if (EKF->CurDDObs.RefPos[0] < 0 && EKF->CurDDObs.RefPos[1] < 0)//�ο���ѡȡʧ��
	{
		EKF->IsInit = false;
		return false;
	}
	//˫��۲�ֵ��˫������
	FormDDObs(&EKF->CurSDObs, &EKF->CurDDObs);
	//��ǰ��Ԫ˫��������
	int Curvalidsat = EKF->CurDDObs.Sats;//�۲�˫��������
	double Curgpscount = EKF->CurDDObs.DDSatNum[0];//�۲�GPS˫��������
	double Curbdscount = EKF->CurDDObs.DDSatNum[1];//�۲�BDS˫��������
	//ǰһʱ����Ԫ˫��������(û��������δ�̶�)
	int validsat = EKF->DDObs.Sats;//��Ч�۲�˫��������
	double gpscount = EKF->DDObs.DDSatNum[0];//��Ч�۲�GPS˫��������
	double bdscount = EKF->DDObs.DDSatNum[1];//��Ч�۲�BDS˫��������
	if (Curvalidsat <= 4)
	{
		EKF->IsInit = false;
		return false;
	}
	//���û�վ������վλ�ó�ֵ
	double BasPos[3] = { 0 };
	double RovPos[3] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		BasPos[i] = Raw->BasEpk.Pos[i];
		RovPos[i] = Rov->Position[i];
	}
	//ȷ����ǰ�۲����������ǰһʱ�̹۲��������
	int tempCurparnum = 3 + 2 * Curvalidsat;
	int parnum = 3 + 2 * validsat;
	double* X_k1 = new double[parnum]();
	double* P_k1 = new double[parnum * parnum]();
	double* tempX = new double[tempCurparnum]();
	double* tempP = new double[tempCurparnum * tempCurparnum]();
	//�˲���ʼ��
	if (EKF->IsInit != true)
	{
		if (InitEKF(&EKF->DDObs, RovPos, parnum, X_k1, P_k1)) EKF->IsInit = true;
	}
	else
	{
		memcpy(X_k1, &EKF->X, parnum * sizeof(double));
		memcpy(P_k1, &EKF->P, parnum * parnum * sizeof(double));
		
	}
	//�˲�ʱ�����
	Prediction(&EKF->DDObs, &EKF->CurDDObs, EKF->Index, tempCurparnum, parnum, X_k1, tempX,P_k1,tempP);
	
	delete []X_k1; delete[]P_k1;
	//���㵱ǰ��ԪGPS��BDS��Ч˫��������
	int sys = 0;
	for (int i0 = 0; i0 < EKF->CurDDObs.Sats; i0++)
	{
		if (EKF->CurDDObs.ddSatObs[i0].System == GPS) sys = 0;
		else if (EKF->CurDDObs.ddSatObs[i0].System == BDS) sys = 1;
		if (EKF->CurDDObs.ddSatObs[i0].Valid != true)
		{
			if (sys == 0)Curgpscount--; else Curbdscount--;
			Curvalidsat--;
		}
	}
	//���µ�ǰ�۲��������
	int Curparnum = 3 + 2 * Curvalidsat;
	double* X_kk1 = new double[Curparnum]();
	double* P_kk1 = new double[Curparnum * Curparnum]();
	int index = 0;
	//��tempX��tempP��δ�������ֵ���к���ѹ����X_kk1��P_kk1
	for (int i = 0; i < tempCurparnum; i++)
	{
		//ǰ����ֱ�Ӹ���
		if (i < 3 || EKF->CurDDObs.ddSatObs[(i-3)/2].Valid == true)
		{
			X_kk1[index] = tempX[i];
			int index2 = 0;
			for (int j = 0; j < tempCurparnum; ++j) {
				if (j < 3 || EKF->CurDDObs.ddSatObs[(j - 3) / 2].Valid == true) {
					P_kk1[index * Curparnum + index2] = tempP[i * tempCurparnum + j];
					index2++;
				}
			}
			index++;
		}
	}
	//��Index�б��Ϊ-1��ֵѹ��
	index = 0;
	for (int i = 0; i < EKF->CurDDObs.Sats; i++)
	{
		int tempindex= EKF->Index[i];
		if (EKF->CurDDObs.ddSatObs[i].Valid == true)
		{
			EKF->Index[index] = tempindex;
			index++;
		}
	}
	/*outputMatrixToFile(P_kk1, Curparnum, Curparnum, "outputP.txt");*/
	delete[]tempX; delete[]tempP;
	//��ֵ��ǰ��Ԫ˫��������(û������)
	EKF->CurDDObs.Sats= Curvalidsat;//�۲�˫��������
	EKF->CurDDObs.DDSatNum[0]= Curgpscount;//�۲�GPS˫��������
	EKF->CurDDObs.DDSatNum[1]= Curbdscount;//�۲�BDS˫��������
	DDUpdate(&EKF->CurSDObs, &EKF->CurDDObs, BasPos, &Raw->BasEpk, &Raw->RovEpk, EKF->Index, X_kk1, P_kk1);
	
	memset(&EKF->X, 0, sizeof(&EKF->X)); memset(&EKF->P, 0, sizeof(&EKF->P));
	memcpy(&EKF->X, X_kk1, Curparnum * sizeof(double));
	memcpy(&EKF->P, P_kk1, Curparnum * Curparnum * sizeof(double));
	
	delete[]X_kk1; delete[]P_kk1;
	if (EKF->CurDDObs.IsSuccess != true)
	{
		EKF->CurDDObs.Ratio = 0;//Ratioֵ
		MatrixSubtraction(3, 1, Rov->Position, Raw->BasEpk.Pos, EKF->CurDDObs.dPos);// ��������
		Comp_dEnu(Raw->BasEpk.Pos, Rov->Position, EKF->CurDDObs.dENH);
		EKF->CurDDObs.FixRMS[0] = Rov->SigmaPos;// ����ⶨλ��rms���
	}
	Raw->DDObs = EKF->CurDDObs;
	return true;
}