// SPP.cpp : 此文件包含粗差探测和接收机位置和速度计算。
//
#include "RTK_Structs.h"
// 线性组合探测粗差
//void DetectOutlier(EPOCHOBS* Obs)
//{
//	int satnum = Obs->SatNum;
//	//将上一组组合值临时保存
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
//		//检查该卫星的双频伪距和相位数据是否有效和完整，若不全或为0，将Valid标记为false，continue
//		if (Obs->SatObs[i].P[0] == 0 || Obs->SatObs[i].P[1] == 0 || Obs->SatObs[i].L[0] == 0 || Obs->SatObs[i].L[1] == 0)
//		{
//			Obs->SatObs[i].Valid = false;
//			continue;
//		}
//		//计算当前历元该卫星的GF和MW、PIF组合值
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
//		//从上个历元的MWGF数据中查找该卫星的GF和MW组合值
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
//		//计算当前历元该卫星GF与上一历元对应GF的差值dGF和对应MW平滑值的差值dMW
//		//如果找不到可直接赋值，如果超限就保持原来Combos，如果没有超限就更新Combos
//		if (find)
//		{
//			dGF = GF - coobs[index].GF;
//			dMW = MW - coobs[index].MW;
//			//检查dGF和dMW是否超限，如果超限保持原来Combos
//			if (fabs(dGF) > 0.05 || fabs(dMW) > 3)
//			{
//				Obs->SatObs[i].Valid = false;
//				/*printf("slip detected sat:%d\n", Obs->SatObs[i].Prn);*/
//			}
//			else
//			{
//				//没有超限，更新Combos
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
//			//在前一时刻找不到，表明是新观测到的卫星，直接continue跳过
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
函数名：粗差探测函数
参数：	obs 观测数据
函数功能：对obs粗差探测并在SatObs里标记Valid,计算每颗卫星的（P）IF组合观测值
****************************************************************************
*/
void DetectOutlier(EPOCHOBS* obs)
{
	// 为了不破坏上一历元的组合观测值数组而建立的缓冲区
	MWGF comobs[MAXCHANNUM];// 存放当前历元所算组合值
	for (int i = 0; i < obs->SatNum; i++)
	{
		// 检查每一颗卫星的双频伪距和相位数据是否完整
		if (fabs(obs->SatObs[i].P[0]) < 1e-8 || fabs(obs->SatObs[i].P[1]) < 1e-8 || fabs(obs->SatObs[i].L[0]) < 1e-8 || fabs(obs->SatObs[i].L[1]) < 1e-8)
		{
			obs->SatObs[i].Valid = false;
			continue; //本颗卫星数据残缺，不计算组合观测值且观测值有效性设置为false
		}
		/*
		// 设置截止信噪比
		if (fabs(obs->SatObs[i].cn0[0]) < 40 || fabs(obs->SatObs[i].cn0[1]) < 40)
		{
			obs->SatObs[i].Valid = false;
			continue; //本颗卫星信噪比较低，不计算组合观测值且观测值有效性设置为false
		}
		*/
		// 计算当前历元的该卫星的GF和MW组合值
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

		// 从上个历元的MWGF数据中查找该卫星的GF和MW组合值
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (obs->ComObs[j].Sys == comobs[i].Sys && obs->ComObs[j].Prn == comobs[i].Prn)
			{
				// 找到了则比较二者是否在限差内
				double dGF = fabs(comobs[i].GF - obs->ComObs[j].GF);
				double dMW = fabs(comobs[i].MW - obs->ComObs[j].MW);

				// 没有超限，则标记true并计算MW的平滑值
				if (dGF < 0.05 && dMW < 3)
				{
					obs->SatObs[i].Valid = true;
					comobs[i].MW = (obs->ComObs[j].MW * obs->ComObs[j].n + comobs[i].MW) / (obs->ComObs[j].n + 1);
					comobs[i].n = obs->ComObs[j].n + 1;
				}
				break;
				// 超限，则标记为粗差（而初始化本就是false，故不需处理）
			}
			// 如果在上个历元中没有找到，则不知其是否可用，默认为初始化时的false
			else continue;
		}
	}
	// 将缓冲区组合观测值的拷贝到obs里
	memcpy(obs->ComObs, comobs, sizeof(comobs));
}
/****************************************************************************
  SPP

  目的：单点定位与测速

  编号：501

  参数:
  Epoch			观测数据结构体
  GpsEph		GPS卫星星历
  BDSEph		BDS卫星星历
  Res		    用户定位结果

  返回值：计算成功返回true，少于4颗卫星或者矩阵求逆失败返回false
****************************************************************************/
bool SPP(EPOCHOBS* Epoch, RAWDAT* Raw, PPRESULT* Result)
{
	//结果中需要留存数据
	double RcvPos[3] = { 0 };//首次定位设置为0
	RcvPos[0] = Result->Position[0];
	RcvPos[1] = Result->Position[1];
	RcvPos[2] = Result->Position[2];
	double dt[2] = { 0,0 };//每次迭代后的时间，m
	int validsat = 0;//有效观测卫星数
	int gpscount = 0;//有效观测GPS卫星数
	int bdscount = 0;//有效观测BDS卫星数
	int parnum = 5;/*有效参数个数*/
	double pdop = 0;//PDOP值，空间位置精度因子
	double sigma0 = 0;//验后单位权中误差

	int Iterator = 0;  /*计算迭代次数，小于10次*/
	double dRcv[3] = { 0 };//每次迭代的位置变化量
	//粗差探测
	DetectOutlier(Epoch);
	do {

		ComputeGPSSatOrbitAtSignalTrans(Epoch, Raw->GpsEph, Raw->BdsEph, RcvPos);
		validsat = 0;//有效观测卫星数
		gpscount = 0;//有效观测GPS卫星数
		bdscount = 0;//有效观测BDS卫星数
		//初始化观测矩阵
		double B[MAXCHANNUM * 5] = { 0 };
		double w[MAXCHANNUM * 1] = { 0 };
		for (int i = 0; i < Epoch->SatNum; i++)
		{
			//卫星位置计算失败、观测数据不完整或有粗差，不参与定位计算
			if ((!Epoch->SatPVT[i].Valid) || (!Epoch->SatObs[i].Valid)) continue;
			else validsat++;

			double dPos[3] = { 0 };//每个观测卫星到接收机的距离
			MatrixSubtraction(3, 1, Epoch->SatPVT[i].SatPos, RcvPos, dPos);

			//观测方程线性化,计算B和w矩阵
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
		//少于4颗卫星，不进行定位解算
		if (validsat < 4)
		{
			Result->IsSuccess = false;
			return false;
		}
		//确定参数个数
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

		double* Qxx = new double[parnum * parnum]();//NBB的逆
		double* W = new double[parnum * 1]();
		double* x = new double[parnum]();//单位m
		double* B0 = new double[MAXCHANNUM*parnum]();

		double Bx[MAXCHANNUM*1] = { 0 };
		double v[MAXCHANNUM * 1] = { 0 };
		double vT[MAXCHANNUM * 1] = { 0 };
		double sigma = 0;

		//最小二乘求解
		MatrixTranspose(MAXCHANNUM, 5, B, BT);
		MatrixMultiply_APB(5, MAXCHANNUM, MAXCHANNUM, 5, BT, P, B, NBB0);
		double* NBB = new double[parnum * parnum]();

		double W0[5 * 1] = { 0 };
		MatrixMultiply_APB(5, MAXCHANNUM, MAXCHANNUM, 1, BT, P, w, W0);
		//观测数据存在特殊情况，需要对Nxx0矩阵和W0以及B重构
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
		//定位精度评价
		MatrixMultiply(MAXCHANNUM, parnum, parnum, 1, B0, x, Bx);
		MatrixSubtraction(MAXCHANNUM, 1, Bx, w, v);
		MatrixTranspose(MAXCHANNUM, 1, v, vT);
		MatrixMultiply_APB(1, MAXCHANNUM, MAXCHANNUM, 1, vT, P, v, &sigma);
		sigma0 = sqrt(sigma / (validsat - parnum));
		pdop = sqrt(Qxx[0] + Qxx[parnum + 1] + Qxx[2 * parnum + 2]);
		//改变接收机位置，为下一轮循环做准备
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
		//动态数组释放
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
	
	SPV(Epoch, Result);//单点测速


	XYZToBLH(RcvPos, Result->BLH, R_CGS2K, F_CGS2K);
	Comp_dEnu(Epoch->Pos, Result->Position, Result->dENH);
	return true;
}

/****************************************************************************
  SPV

  目的：单点测速

  编号：503

  参数:
  Epoch			观测数据结构体
  Res		    用户定位结果

  返回值：计算成功或失败
****************************************************************************/
void SPV(EPOCHOBS* Epoch, PPRESULT* Result)
{
	int validsat = 0;//有效卫星数
	double RcvPos[3] = { 0 };
	RcvPos[0] = Result->Position[0];
	RcvPos[1] = Result->Position[1];
	RcvPos[2] = Result->Position[2];

	double sigma0 = 0;
	double x[4 * 1] = { 0 };//单位m
	//初始化观测矩阵
	double B[MAXCHANNUM * 4] = { 0 };
	double w[MAXCHANNUM * 1] = { 0 };
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if ((!Epoch->SatPVT[i].Valid) || (!Epoch->SatObs[i].Valid)) continue;
		else validsat++;

		double dPos[3] = { 0 };//每个观测卫星到接收机的距离
		MatrixSubtraction(3, 1, Epoch->SatPVT[i].SatPos, RcvPos, dPos);

		//观测方程线性化,计算B和w矩阵
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

	//最小二乘求解并精度评定
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

	//赋值给定位结果结构体
	Result->Velocity[0] = x[0];
	Result->Velocity[1] = x[1];
	Result->Velocity[2] = x[2];
	Result->RcvClkSft = x[3];
	Result->SigmaVel = sigma0;
}