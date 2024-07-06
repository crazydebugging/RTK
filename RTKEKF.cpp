// RTKEKF.cpp : 此文件包含RTK卡尔曼滤波计算。
//
#include "RTK_Structs.h"
/****************************************************************************
  InitEKF

  目的：滤波初始化(待估参数和状态协方差)


  编号：701

  参数:

****************************************************************************/
bool InitEKF(DDCOBS* DDObs, double* RovPos,int& parnum, double* X, double* P)
{
	int sys = 0;
	//位置赋初值
	for (int i = 0; i < 3; i++)
	{
		X[i] = RovPos[i];
		P[i * parnum + i] = 10;
	}
	//N赋初值
	int i = 3;//实际行数
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

  目的：时间预测


  编号：702

  参数:

****************************************************************************/
void Prediction(DDCOBS* DDObs, DDCOBS* CurDDObs, int* Index,int& Curparnum, int& parnum,double*X0, double* X1,double*P0,double*P1)
{
	double* phi = new double[Curparnum * parnum]();
	double* Q = new double[Curparnum * Curparnum]();
	//索引传递
	int index[MAXCHANNUM] = { 0 };
	for (int i = 0; i < DDObs->Sats; i++)
	{
		index[i] = Index[i];
	}
	for (int i = 0; i < MAXCHANNUM; i++) Index[i]= 0;
	//位置传递
	for (int i = 0; i < 3; i++)
	{
		phi[i* parnum + i] = 1;
	}
	//参考星改变时
	int find = 0;
	int prnindex = 0;
	//模糊度传递,传递成功的卫星新的Index为1，失败为0
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
				//参考星未改变时
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
				//参考星改变时
				else if (Cursys == sys && CurDDObs->RefPrn[Cursys] != DDObs->RefPrn[sys])
				{
					//对每一颗当前历元卫星，遍历上个历元卫星
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
			//参考星改变并找到时,遍历当前卫星
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
			//参考星改变但没找到，当前历元卫星全部找不到，但如果是参考星未改变，则无需此判断条件
			else if(Index[i]!=1) Index[i] = 0;
		}
	}
	MatrixMultiply(Curparnum, parnum, parnum, 1, phi, X0, X1);
	//对Q初始化
	for (int i = 0; i <Curparnum; i++)
	{
		if(i<3)	Q[i * Curparnum + i] = 1e-8;
		else Q[i * Curparnum + i] = 1e-10;
	}
	MatrixMultiply_ABAT_C(Curparnum, parnum, parnum, Curparnum, phi, P0, Q, P1);
	//周跳探测,对双差观测值。利用上一历元传递的N探测粗差，并对所有有周跳值进行标记，并把有周跳的值标记为-1
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
	//对Xkk1中未成功传递的固定模糊度进行初始化（一般是因为这个历元有新升起卫星或者出现粗差）状态和状态协方差阵
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

  目的：测量更新,不能使用有周跳的观测及其对应的模糊度

  编号：703

  参数:

****************************************************************************/
//void Update(SDEPOCHOBS* CurSDObs, DDCOBS* CurDDObs, double* BasPos, EPOCHOBS* BasEpk, EPOCHOBS* RovEpk,int* Index,double* X0, double* X1, double* P0, double* P1)
//{
//
//}
/****************************************************************************
  DDUpdate

  目的：利用双差观测值测量更新,不能使用有周跳的观测及其对应的模糊度

  编号：703

  参数:

****************************************************************************/
bool DDUpdate(SDEPOCHOBS* CurSDObs, DDCOBS* CurDDObs, double* BasPos,EPOCHOBS*BasEpk, EPOCHOBS* RovEpk, int* Index, double* X1,double* P1)
{
	//计算GPS和BDS双差卫星数(无周跳)
	int validsat = CurDDObs->Sats;//观测双差卫星数
	double gpscount = CurDDObs->DDSatNum[0];//观测GPS双差卫星数
	double bdscount = CurDDObs->DDSatNum[1];//观测BDS双差卫星数
	//确定参数个数
	int parnum = 3 + 2 * validsat;
	double sigma0 = 0;//验后单位权中误差
	double pdop = 0;//PDOP值，空间位置精度因子
	//使用时间预测结果进行递推
	double RovPos[3] = { 0 };
	double* N = new double[validsat * 2]();//双差浮点解模糊度，周
	for (int i = 0; i < parnum; i++)
	{
		if(i<3)RovPos[i] = X1[i];
		else N[i - 3] = X1[i];
	}
	//计算基站坐标到所有单差卫星的几何距离
	double BasSat[MAXCHANNUM] = { 0 };
	for (int i = 0; i < CurSDObs->SatNum; i++)
	{
		int BasIndex = CurSDObs->SdSatObs[i].nBas;
		if (CurSDObs->SdSatObs[i].Valid == true)//单差数据有效并卫星位置计算成功
		{
			double dPos[3] = { 0 };//每个观测卫星到基站的距离
			MatrixSubtraction(3, 1, BasEpk->SatPVT[BasIndex].SatPos, BasPos, dPos);
			BasSat[i] = Norm(3, dPos);
		}
	}
	//计算流动站到参考星的几何距离
	double RovRefSat[2] = { 0 };
	for (int i = 0; i < 2; i++)
	{
		if (CurDDObs->RefPos[i] >= 0)//参考星选取成功
		{
			int RovIndex = CurSDObs->SdSatObs[CurDDObs->RefPos[i]].nRov;
			double dPos[3] = { 0 };//每个观测卫星到基站的距离
			MatrixSubtraction(3, 1, RovEpk->SatPVT[RovIndex].SatPos, RovPos, dPos);
			RovRefSat[i] = Norm(3, dPos);
		}
	}
	//计算权矩阵;线性化双差观测方程得到B矩阵和W向量,参考星不用计算
	double* B = new double[(validsat * 4) * parnum]();
	double* w = new double[(validsat * 4)]();
	double* R = new double[(validsat * 4) * (validsat * 4)]();
	int i = 0;//实际矩阵中行数
	int sys = 0;
	for (int i0 = 0; i0 < CurDDObs->Sats; i0++)//对双差观测值进行循环
	{
		if (CurDDObs->ddSatObs[i0].System == GPS) sys = 0;
		else if (CurDDObs->ddSatObs[i0].System == BDS) sys = 1;

		if (CurDDObs->ddSatObs[i0].Valid != true) continue;
		int di = CurDDObs->ddSatObs[i0].Pos;//当前双差卫星在单差卫星中的位置
		int RovIndex = CurSDObs->SdSatObs[di].nRov;//当前单差卫星在流动站观测中的位置
		int RovRefIndex = CurDDObs->RefPos[sys];//参考卫星在单差中的序号
		double dPos[3] = { 0 };//每个双差观测卫星到流动站的距离
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
	//打印B矩阵和W向量,权矩阵
		//outputMatrixToFile(B, (validsat * 4), (3 + 2 * validsat), "outputB.txt");
		//outputMatrixToFile(w, (validsat * 4), 1, "outputw.txt");
		//outputMatrixToFile(R, (validsat * 4), (validsat * 4), "outputR.txt");
	if (validsat < 4)
	{
		CurDDObs->IsSuccess = false;
		return false;
	}
	//滤波测量更新
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
	//定位精度评价
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
	//动态数组释放
	delete[]B;delete[]w;delete[]R;delete[]BPR;delete[]NBPR;delete[]BT;
	delete[]K; delete[]Kw; delete[]KB; delete[]I; delete[]IKB; delete[]IPI; delete[]KRK;
	//保存双差浮点解模糊度及其协因数矩阵，用于LAMBDA模糊度固定
	double* Qnn = new double[(2 * validsat) * (2 * validsat)]();//双差浮点解模糊度协因数矩阵
	for (int i = 0; i < 2 * validsat; i++) {
		N[i] = X1[3 + i];
		for (int j = 0; j < 2 * validsat; j++) {
			Qnn[i * (2 * validsat) + j] = P1[(parnum - 2 * validsat + i) * parnum + (parnum - 2 * validsat + j)];
		}
	}
	CurDDObs->IsSuccess = true;
	//LAMBDA模糊度固定
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
		MatrixSubtraction(3, 1, RovPos, BasPos, CurDDObs->dPos);// 基线向量
		Comp_dEnu(BasPos, RovPos, CurDDObs->dENH);
		CurDDObs->FixRMS[0] = sigma0 * pdop;// 浮点解定位中rms误差
		return false;
	}
	else
	{
		CurDDObs->bFixed = true;
		for (int i = 0; i < 2 * validsat; i++) {
			X1[3 + i]= CurDDObs->FixedAmb[i];
		}
		CurDDObs->Ratio = CurDDObs->ResAmb[1] / CurDDObs->ResAmb[0];//Ratio值
		for (int i = 0; i < CurDDObs->Sats; i++)
		{
			if (Index[i] == 0) Index[i] = 1;
		}
		for (int i = 0; i < 3; i++)
		{
			RovPos[i] = X1[i];
		}
		MatrixSubtraction(3, 1, RovPos, BasPos, CurDDObs->dPos);// 基线向量
		Comp_dEnu(BasPos, RovPos, CurDDObs->dENH);
		CurDDObs->FixRMS[0] = sigma0 * pdop;// 浮点解定位中rms误差
	}
	delete[]N;delete[]Qnn;
	return true;
}
/****************************************************************************
  RTKekf

  目的：双差相对定位浮点解(卡尔曼滤波版)


  编号：704

  输入参数:
  Raw			RTK定位数据，有原始观测值（有卫星位置计算等中间结果）、单差观测值、双差基准星选取结果、星历
  Rov			流动站SPP定位结果

  返回值：浮点解解算是否成功，计算成功返回true，少于4颗卫星或者矩阵求逆失败返回false
****************************************************************************/
bool RTKekf(RAWDAT* Raw, PPRESULT* Rov, RTKEKF*EKF)
{
	#pragma omp parallel for  // 使用OpenMP进行并行计算
	EKF->DDObs = EKF->CurDDObs;//在同步前将上一轮原始观测值保留
	//观测值
	DetectLockTimeHalf(&Raw->BBasEpk, &Raw->BasEpk); DetectLockTimeHalf(&Raw->BRovEpk, &Raw->RovEpk);//利用接收机的质量数据Locktime检验周跳情况
	//单差观测值及单差卫星
	FormSDEpochObs(&Raw->RovEpk, &Raw->BasEpk, &EKF->CurSDObs);//站间单差观测值计算
	DetectCycleSlip(&EKF->CurSDObs);//对站间单差观测值，用MW和GF组合探测周跳
	DetRefSat(&Raw->RovEpk, &Raw->BasEpk, &EKF->CurSDObs, &EKF->CurDDObs);//选取基准星
	if (EKF->CurDDObs.RefPos[0] < 0 && EKF->CurDDObs.RefPos[1] < 0)//参考星选取失败
	{
		EKF->IsInit = false;
		return false;
	}
	//双差观测值及双差卫星
	FormDDObs(&EKF->CurSDObs, &EKF->CurDDObs);
	//当前历元双差卫星数
	int Curvalidsat = EKF->CurDDObs.Sats;//观测双差卫星数
	double Curgpscount = EKF->CurDDObs.DDSatNum[0];//观测GPS双差卫星数
	double Curbdscount = EKF->CurDDObs.DDSatNum[1];//观测BDS双差卫星数
	//前一时刻历元双差卫星数(没有周跳或未固定)
	int validsat = EKF->DDObs.Sats;//有效观测双差卫星数
	double gpscount = EKF->DDObs.DDSatNum[0];//有效观测GPS双差卫星数
	double bdscount = EKF->DDObs.DDSatNum[1];//有效观测BDS双差卫星数
	if (Curvalidsat <= 4)
	{
		EKF->IsInit = false;
		return false;
	}
	//设置基站和流动站位置初值
	double BasPos[3] = { 0 };
	double RovPos[3] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		BasPos[i] = Raw->BasEpk.Pos[i];
		RovPos[i] = Rov->Position[i];
	}
	//确定当前观测参数个数和前一时刻观测参数个数
	int tempCurparnum = 3 + 2 * Curvalidsat;
	int parnum = 3 + 2 * validsat;
	double* X_k1 = new double[parnum]();
	double* P_k1 = new double[parnum * parnum]();
	double* tempX = new double[tempCurparnum]();
	double* tempP = new double[tempCurparnum * tempCurparnum]();
	//滤波初始化
	if (EKF->IsInit != true)
	{
		if (InitEKF(&EKF->DDObs, RovPos, parnum, X_k1, P_k1)) EKF->IsInit = true;
	}
	else
	{
		memcpy(X_k1, &EKF->X, parnum * sizeof(double));
		memcpy(P_k1, &EKF->P, parnum * parnum * sizeof(double));
		
	}
	//滤波时间更新
	Prediction(&EKF->DDObs, &EKF->CurDDObs, EKF->Index, tempCurparnum, parnum, X_k1, tempX,P_k1,tempP);
	
	delete []X_k1; delete[]P_k1;
	//计算当前历元GPS和BDS有效双差卫星数
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
	//更新当前观测参数个数
	int Curparnum = 3 + 2 * Curvalidsat;
	double* X_kk1 = new double[Curparnum]();
	double* P_kk1 = new double[Curparnum * Curparnum]();
	int index = 0;
	//将tempX和tempP中未遇到标记值的行和列压缩到X_kk1和P_kk1
	for (int i = 0; i < tempCurparnum; i++)
	{
		//前三行直接复制
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
	//将Index中标记为-1的值压缩
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
	//赋值当前历元双差卫星数(没有周跳)
	EKF->CurDDObs.Sats= Curvalidsat;//观测双差卫星数
	EKF->CurDDObs.DDSatNum[0]= Curgpscount;//观测GPS双差卫星数
	EKF->CurDDObs.DDSatNum[1]= Curbdscount;//观测BDS双差卫星数
	DDUpdate(&EKF->CurSDObs, &EKF->CurDDObs, BasPos, &Raw->BasEpk, &Raw->RovEpk, EKF->Index, X_kk1, P_kk1);
	
	memset(&EKF->X, 0, sizeof(&EKF->X)); memset(&EKF->P, 0, sizeof(&EKF->P));
	memcpy(&EKF->X, X_kk1, Curparnum * sizeof(double));
	memcpy(&EKF->P, P_kk1, Curparnum * Curparnum * sizeof(double));
	
	delete[]X_kk1; delete[]P_kk1;
	if (EKF->CurDDObs.IsSuccess != true)
	{
		EKF->CurDDObs.Ratio = 0;//Ratio值
		MatrixSubtraction(3, 1, Rov->Position, Raw->BasEpk.Pos, EKF->CurDDObs.dPos);// 基线向量
		Comp_dEnu(Raw->BasEpk.Pos, Rov->Position, EKF->CurDDObs.dENH);
		EKF->CurDDObs.FixRMS[0] = Rov->SigmaPos;// 浮点解定位中rms误差
	}
	Raw->DDObs = EKF->CurDDObs;
	return true;
}