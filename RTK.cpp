// RTK.cpp : 此文件包含RTK浮点解与固定解计算。
//
#include "RTK_Structs.h"
#define limt 0.1
#define limT 3

/****************************************************************************
  GetSynObs

  目的：数据解码与时间同步，更新RTK定位数据中原始观测数据和星历


  编号：601

  参数:
  FBas			基站文件指针
  FRov			移动站文件指针
  BasSock		基站网络端口
  RovSock		移动站网络端口
  Raw			RTK定位数据

  返回值：同步成功返回1，同步失败返回0
****************************************************************************/
int GetSynObs(FILE* FBas, FILE* FRov, SOCKET& BasSock,SOCKET& RovSock, RAWDAT* Raw)
{
	Raw->BBasEpk = Raw->BasEpk; Raw->BRovEpk = Raw->RovEpk; Raw->BDDObs = Raw->DDObs; //在同步前将上一轮原始观测值保留
	if (FBas!=NULL&&(!(feof(FBas) || feof(FRov))))
	{
		static unsigned char Basbuff[MAXRAWLEN], Rovbuff[MAXRAWLEN];
		static int Blen = 0, Rlen = 0;

		int dt = 0;
		int Rresult = 0, Bresult = 0;
		while (Rresult != 1) {
			//保存上一轮循环中buff的剩余值
			int Rtemp = Rlen;
			// 从文件中读取数据到buff
			Rlen = fread(Rovbuff + Rtemp, sizeof(unsigned char), MAXRAWLEN - Rtemp, FRov);
			Rlen += Rtemp;
			// 解码数据
			Rresult = DecodeNovOem7Dat(Rovbuff, Rlen, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph,1);
		}
		do {
			if (dt < -limt) {
				Rresult = 0;
				while (Rresult != 1) {
					//保存上一轮循环中buff的剩余值
					int Rtemp = Rlen;
					// 从文件中读取数据到buff
					Rlen = fread(Rovbuff + Rtemp, sizeof(unsigned char), MAXRAWLEN - Rtemp, FRov);
					Rlen += Rtemp;
					// 解码数据
					Rresult = DecodeNovOem7Dat(Rovbuff, Rlen, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph,1);
				}
			}
			dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
			if (fabs(dt) <= limt)	break;
			if (dt > limt) {
				Bresult = 0;
				while (Bresult != 1) {
					//保存上一轮循环中buff的剩余值
					int Btemp = Blen;
					// 从文件中读取数据到buff
					Blen = fread(Basbuff + Btemp, sizeof(unsigned char), MAXRAWLEN - Btemp, FBas);
					Blen += Btemp;
					// 解码数据
					Bresult = DecodeNovOem7Dat(Basbuff, Blen, &(Raw->BasEpk), Raw->GpsEph, Raw->BdsEph,1);
				}
			}
			dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
			if (fabs(dt) <= limt)	break;
		} while (1);
		return 1;
	}
	else
	{
		//存储变量
		static unsigned char BasBuff[3 * MAXRAWLEN], RovBuff[3 * MAXRAWLEN];
		static int BlenD = 0, RlenD = 0;
		unsigned char Basbuff[MAXRAWLEN], Rovbuff[MAXRAWLEN];
		int BlenR = 0, RlenR = 0;
		//循环标志
		int dt = 0;
		int Rresult = 0, Bresult = 0;
		Sleep(980);
		if ((RlenR = recv(RovSock, (char*)Rovbuff, MAXRAWLEN, 0)) > 0) {
			/*printf("%d\n", RlenR);*/
			memcpy(RovBuff + RlenD, Rovbuff, RlenR);
			RlenD += RlenR;
			memset(Rovbuff, 0, MAXRAWLEN);
			//while (Rresult != 5) {
			//	// 解码数据
			//	Rresult = DecodeNovOem7Dat(RovBuff, RlenD, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph,0);
			//}
			Rresult = DecodeNovOem7Dat(RovBuff, RlenD, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph, 0);
		}
		if ((BlenR = recv(BasSock, (char*)Basbuff, MAXRAWLEN, 0)) > 0) {
			memcpy(BasBuff + BlenD, Basbuff, BlenR);
			BlenD += BlenR;
			memset(Basbuff, 0, MAXRAWLEN);
			//while (Bresult != 5) {
			//	// 解码数据
			//	Bresult = DecodeNovOem7Dat(BasBuff, BlenD, &(Raw->BasEpk), Raw->GpsEph, Raw->BdsEph,0);
			//}
			Bresult = DecodeNovOem7Dat(BasBuff, BlenD, &(Raw->BasEpk), Raw->GpsEph, Raw->BdsEph, 0);
		}
		dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
		if (fabs(dt) <= limT)	return 1;
		else return 0;
		//int flag = 0;
		//do {
		//	if ((RlenR = recv(RovSock, (char*)Rovbuff, MAXRAWLEN, 0)) > 0) {
		//		memcpy(RovBuff + RlenD, Rovbuff, RlenR);
		//		RlenD += RlenR;
		//		memset(Rovbuff, 0, MAXRAWLEN);
		//		Rresult = 0;
		//		while (Rresult != 5) {
		//			// 解码数据
		//			Rresult = DecodeNovOem7Dat(Rovbuff, RlenD, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph);
		//			if (Rresult == 1) flag = 1;
		//		}
		//	}
		//} while (flag != 1);
		//do {
		//	if (dt < limT) {
		//		flag = 0;
		//		do {
		//			if ((RlenR = recv(RovSock, (char*)Rovbuff, MAXRAWLEN, 0)) > 0) {
		//				memcpy(RovBuff + RlenD, Rovbuff, RlenR);
		//				RlenD += RlenR;
		//				memset(Rovbuff, 0, MAXRAWLEN);
		//				Rresult = 0;
		//				while (Rresult != 5) {
		//					// 解码数据
		//					Rresult = DecodeNovOem7Dat(RovBuff, RlenD, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph);
		//					if (Rresult == 1) flag = 1;
		//				}
		//			}
		//		}while(flag != 1);
		//	}
		//	dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
		//	if (fabs(dt) <= limT)	break;
		//	if (dt > limT) {
		//		flag = 0;
		//		do {
		//			if ((BlenR = recv(BasSock, (char*)Basbuff, MAXRAWLEN, 0)) > 0) {
		//				memcpy(BasBuff + BlenD, Basbuff, BlenR);
		//				BlenD += BlenR;
		//				memset(Basbuff, 0, MAXRAWLEN);
		//				Bresult = 0;
		//				while (Bresult != 5) {
		//					// 解码数据
		//					Bresult = DecodeNovOem7Dat(BasBuff, BlenD, &(Raw->BasEpk), Raw->GpsEph, Raw->BdsEph);
		//					if (Bresult == 1) flag = 1;
		//				}
		//			}
		//		} while(flag != 1);
		//	}
		//	dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
		//	if (fabs(dt) <= limT)	break;
		//} while (1);
		//return 1;
	}

}
/****************************************************************************
  DetectLockTimeHalf

  目的：利用接收机的质量数据Locktime和Parity，检验周跳和半周情况,如果出现半周或者周跳将观测数据Valid=false


  编号：602

  参数:
  Obs			前一时刻观测值数据
  CurObs		当前时刻观测值数据

****************************************************************************/
void DetectLockTimeHalf(const EPOCHOBS* Obs, EPOCHOBS* CurObs)
{
	int satnum = CurObs->SatNum;
	for (int i = 0; i < satnum; i++)
	{
		GNSSSys sys = CurObs->SatObs[i].System;
		int prn = CurObs->SatObs[i].Prn;
		//从上个历元的每颗卫星的观测数据中查找该卫星值
		int find = 0;
		int index = 0;
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (sys == Obs->SatObs[j].System && prn == Obs->SatObs[j].Prn)
			{
				find = 1;
				index = j;
				break;
			}
		}
		for (int flag = 0; flag < 2; flag++)
		{
			//检查当前时刻该卫星的双频相位数据是否有效和完整
			if (CurObs->SatObs[i].L[flag] != 0)
			{
				//判断相位数据是否存在半周
				if (CurObs->SatObs[i].half[flag] == 0)
				{
					CurObs->SatObs[i].Valid = false;
					/*printf("half cycle detected sat:%d\n", CurObs->SatObs[i].Prn);*/
				}
			}
			//检查该卫星的双频相位数据是否有效和完整，若不全或为0，continue
			if (Obs->SatObs[index].L[flag] != 0 && CurObs->SatObs[i].L[flag] != 0)
			{
				//比较当前历元该卫星Locktime与上一历元对应Locktime的大小,判断历元间相位数据的连续性
				if (find)
				{
					if (CurObs->SatObs[i].LockTime[flag] < Obs->SatObs[index].LockTime[flag])
					{
						CurObs->SatObs[i].Valid = false;
						/*printf("cycle slip detected sat:%d\n", CurObs->SatObs[i].Prn);*/
					}
				}
			}
		}
	}
}
/****************************************************************************
   FormSDEpochObs

  目的：站间单差观测值计算


  编号：603

  参数:
  EpkA			移动站观测数据
  EpkB			基站观测数据
  SDObs			单差观测数据

****************************************************************************/
void FormSDEpochObs(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs)
{
	memset(&SDObs->SdSatObs, 0, sizeof(SDObs->SdSatObs));// 使用memset()清空单差结构体中的观测数据
	//对相同卫星、相同频率、相同类型观测值进行站间求差
	SDObs->Time = EpkA->Time;
	unsigned long satsum = EpkA->SatNum;
	unsigned short satcount = 0;//移动站卫星计数
	unsigned short n = 0;//单差卫星计数
	while (satcount < satsum)
	{
		GNSSSys sys = EpkA->SatObs[satcount].System;
		unsigned short prn = EpkA->SatObs[satcount].Prn;//卫星号和系统
		//流动站观测的卫星号和系统，在基站观测到的数据中是否存在，存在就返回当前的数组下标
		int find = 0;
		int index = 0;
		for (int i = 0; i < MAXCHANNUM; i++)
		{
			if (EpkB->SatObs[i].Prn == prn && EpkB->SatObs[i].System == sys)
			{
				find = 1;
				index = i;
				break;
			}
		}
		if (find)
		{
			SDObs->SdSatObs[n].Prn = prn;
			SDObs->SdSatObs[n].System = sys;
			SDObs->SdSatObs[n].nBas = index;
			SDObs->SdSatObs[n].nRov = satcount;
			//检查该卫星的移动站和基站数据是否有效和完整，若不全或为0，continue
			if (EpkA->SatObs[satcount].Valid == true && EpkB->SatObs[index].Valid == true)
			{
				for (int f = 0; f < 2; f++)
				{
					if (EpkA->SatObs[satcount].P[f] != 0 && EpkB->SatObs[index].P[f] != 0)	SDObs->SdSatObs[n].dP[f] = EpkA->SatObs[satcount].P[f] - EpkB->SatObs[index].P[f];
					if (EpkA->SatObs[satcount].L[f] != 0 && EpkB->SatObs[index].L[f] != 0)	SDObs->SdSatObs[n].dL[f] = EpkA->SatObs[satcount].L[f] - EpkB->SatObs[index].L[f];
				}
			}
			n++;
		}
		satcount++;
	}
	SDObs->SatNum = n;
}
/****************************************************************************
  DetectCycleSlip

  目的：对站间单差观测值，用MW和GF组合探测周跳


  编号：604

  参数:
  Obs			单差观测数据
****************************************************************************/
void DetectCycleSlip(SDEPOCHOBS* Obs)
{
	// 为了不破坏上一历元的组合观测值数组而建立的缓冲区
	MWGF comobs[MAXCHANNUM];// 存放当前历元所算组合值
	for (int i = 0; i < Obs->SatNum; i++)
	{
		// 检查每一颗卫星的双频伪距和相位数据是否完整
		if (fabs(Obs->SdSatObs[i].dP[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[1]) < 1e-8 || fabs(Obs->SdSatObs[i].dL[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dL[1]) < 1e-8)
		{
			Obs->SdSatObs[i].Valid = false;
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
		comobs[i].Sys = Obs->SdSatObs[i].System;
		comobs[i].Prn = Obs->SdSatObs[i].Prn;
		comobs[i].GF = Obs->SdSatObs[i].dL[0] - Obs->SdSatObs[i].dL[1];
		if (comobs[i].Sys == GPS)
		{
			comobs[i].MW = (FG1_GPS * Obs->SdSatObs[i].dL[0] - FG2_GPS * Obs->SdSatObs[i].dL[1]) / (FG1_GPS - FG2_GPS) -
				(FG1_GPS * Obs->SdSatObs[i].dP[0] + FG2_GPS * Obs->SdSatObs[i].dP[1]) / (FG1_GPS + FG2_GPS);
			comobs[i].PIF = (FG1_GPS * FG1_GPS * Obs->SdSatObs[i].dP[0] - FG2_GPS * FG2_GPS * Obs->SdSatObs[i].dP[1]) / (FG1_GPS * FG1_GPS - FG2_GPS * FG2_GPS);
		}
		else if (comobs[i].Sys == BDS)
		{
			comobs[i].MW = (FG1_BDS * Obs->SdSatObs[i].dL[0] - FG3_BDS * Obs->SdSatObs[i].dL[1]) / (FG1_BDS - FG3_BDS) -
				(FG1_BDS * Obs->SdSatObs[i].dP[0] + FG3_BDS * Obs->SdSatObs[i].dP[1]) / (FG1_BDS + FG3_BDS);
			comobs[i].PIF = (FG1_BDS * FG1_BDS * Obs->SdSatObs[i].dP[0] - FG3_BDS * FG3_BDS * Obs->SdSatObs[i].dP[1]) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
		}

		comobs[i].n = 1;

		// 从上个历元的MWGF数据中查找该卫星的GF和MW组合值
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (Obs->SdCObs[j].Sys == comobs[i].Sys && Obs->SdCObs[j].Prn == comobs[i].Prn)
			{
				// 找到了则比较二者是否在限差内
				double dGF = fabs(comobs[i].GF - Obs->SdCObs[j].GF);
				double dMW = fabs(comobs[i].MW - Obs->SdCObs[j].MW);

				// 没有超限，则标记true并计算MW的平滑值
				if (dGF < 0.05 && dMW < 3)
				{
					Obs->SdSatObs[i].Valid = true;
					comobs[i].MW = (Obs->SdCObs[j].MW * Obs->SdCObs[j].n + comobs[i].MW) / (Obs->SdCObs[j].n + 1);
					comobs[i].n = Obs->SdCObs[j].n + 1;
				}
				break;
				// 超限，则标记为粗差（而初始化本就是false，故不需处理）
			}
			else continue;
		}
		// 如果在上个历元中没有找到，则不知其是否可用，默认为初始化时的false
	}
	// 将缓冲区组合观测值的拷贝到obs里
	memcpy(Obs->SdCObs, comobs, sizeof(comobs));
}
/****************************************************************************
  DetRefSat

  目的：选取基准星


  编号：605

  参数:
  EpkA			移动站观测数据
  EpkB			基站观测数据
  SDObs			单差观测数据
  DDObs			双差相关的数据
****************************************************************************/
void DetRefSat(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
	//对各卫星导航系统，分别选取一颗观测值数据质量高的卫星作为参考星，其它卫星的单差观测值与该参考星对应的单差观测值求差
	//参考星选取要求:伪距和载波相位通过周跳探测，没有粗差和周跳标记;卫星星历正常，卫星位置计算成功;高度角大或CN0大
	int RefPrn[2] = { -1, -1 }, RefPos[2] = { -1, -1 };         // 参考星卫星号与单差观测中的存储位置，0=GPS; 1=BDS
	double MAXElev[2] = { 0,0 };	  //最大卫星高度角，0=GPS; 1=BDS
	//对每个单差卫星
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		int RovIndex = SDObs->SdSatObs[i].nRov; int BasIndex = SDObs->SdSatObs[i].nBas;
		//单差数据有效并卫星位置计算成功
		if (SDObs->SdSatObs[i].Valid == true && EpkA->SatPVT[RovIndex].Valid == true && EpkB->SatPVT[BasIndex].Valid == true)
		{
			SDObs->SdSatObs[i].Valid = true;
			if (SDObs->SdSatObs[i].System == GPS)
			{
				if (MAXElev[0] < EpkA->SatPVT[RovIndex].Elevation)
				{
					MAXElev[0] = EpkA->SatPVT[RovIndex].Elevation;
					RefPrn[0] = SDObs->SdSatObs[i].Prn;
					RefPos[0] = i;
				}
			}
			else if (SDObs->SdSatObs[i].System == BDS)
			{
				if (MAXElev[1] < EpkA->SatPVT[RovIndex].Elevation)
				{
					MAXElev[1] = EpkA->SatPVT[RovIndex].Elevation;
					RefPrn[1] = SDObs->SdSatObs[i].Prn;
					RefPos[1] = i;
				}
			}
		}
		else SDObs->SdSatObs[i].Valid = false;
	}
	for (int i = 0; i < 2; i++) {
		DDObs->RefPrn[i] = RefPrn[i];
		DDObs->RefPos[i] = RefPos[i];
	}
}
/****************************************************************************
   FormDDObs

  目的：星间双差观测值计算


  编号：606

  参数:
  SDObs			单差观测数据
  DDObs			双差相关数据

****************************************************************************/
void FormDDObs(const SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
	memset(&DDObs->ddSatObs, 0, sizeof(DDObs->ddSatObs));// 使用memset()清空单差结构体中的观测数据
	//对相同卫星、相同频率、相同类型观测值进行星间求差
	DDObs->Time = SDObs->Time;
	//计算GPS和BDS双差卫星数
	int validsat = 0;//观测双差卫星数
	int gpscount = 0;//观测GPS双差卫星数
	int bdscount = 0;//观测BDS双差卫星数
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		GNSSSys system = SDObs->SdSatObs[i].System;
		int sys = 0;
		if (system == GPS)
		{
			sys = 0;
			if (SDObs->SdSatObs[i].Prn != DDObs->RefPrn[0] && SDObs->SdSatObs[i].Valid == true) gpscount++;
			else continue;
		}
		else if (system == BDS)
		{
			sys = 1;
			if (SDObs->SdSatObs[i].Prn != DDObs->RefPrn[1] && SDObs->SdSatObs[i].Valid == true) bdscount++;
			else continue;
		}
		DDObs->ddSatObs[validsat].Prn = SDObs->SdSatObs[i].Prn;
		DDObs->ddSatObs[validsat].System = system;
		DDObs->ddSatObs[validsat].Pos = i;
		int SdRefIndex = DDObs->RefPos[sys];//参考卫星在单差中的序号
		for (int j = 0; j < 2; j++)
		{
			DDObs->ddSatObs[validsat].ddP[j] = SDObs->SdSatObs[i].dP[j] - SDObs->SdSatObs[SdRefIndex].dP[j];
			DDObs->ddSatObs[validsat].ddL[j] = SDObs->SdSatObs[i].dL[j] - SDObs->SdSatObs[SdRefIndex].dL[j];
		}
		validsat++;
	}
	DDObs->DDSatNum[0] = gpscount;
	DDObs->DDSatNum[1] = bdscount;
	DDObs->Sats = validsat;
}
/****************************************************************************
  DDetectCycleSlip

  目的：对站星双差观测值，用MW和GF组合探测周跳


  编号：607

  参数:
  Obs			前一时刻观测值数据
  CurObs		当前时刻观测值数据
****************************************************************************/
void DDetectCycleSlip(DDCOBS* Obs)
{
	// 为了不破坏上一历元的组合观测值数组而建立的缓冲区
	MWGF comobs[MAXCHANNUM];// 存放当前历元所算组合值

	for (int i = 0; i < Obs->Sats; i++)
	{
		// 检查每一颗卫星的双频伪距和相位数据是否完整
		if (fabs(Obs->ddSatObs[i].ddP[0]) < 1e-8 || fabs(Obs->ddSatObs[i].ddP[1]) < 1e-8 || fabs(Obs->ddSatObs[i].ddL[0]) < 1e-8 || fabs(Obs->ddSatObs[i].ddL[1]) < 1e-8)
		{
			Obs->ddSatObs[i].Valid = false;
			continue; //本颗卫星数据残缺，不计算组合观测值且观测值有效性设置为false
		}
		// 计算当前历元的该卫星的GF和MW组合值
		comobs[i].Sys = Obs->ddSatObs[i].System;
		comobs[i].Prn = Obs->ddSatObs[i].Prn;
		comobs[i].GF = Obs->ddSatObs[i].ddL[0] - Obs->ddSatObs[i].ddL[1];
		if (comobs[i].Sys == GPS)
		{
			comobs[i].MW = (FG1_GPS * Obs->ddSatObs[i].ddL[0] - FG2_GPS * Obs->ddSatObs[i].ddL[1]) / (FG1_GPS - FG2_GPS) -
				(FG1_GPS * Obs->ddSatObs[i].ddP[0] + FG2_GPS * Obs->ddSatObs[i].ddP[1]) / (FG1_GPS + FG2_GPS);
			comobs[i].PIF = (FG1_GPS * FG1_GPS * Obs->ddSatObs[i].ddP[0] - FG2_GPS * FG2_GPS * Obs->ddSatObs[i].ddP[1]) / (FG1_GPS * FG1_GPS - FG2_GPS * FG2_GPS);
		}
		else if (comobs[i].Sys == BDS)
		{
			comobs[i].MW = (FG1_BDS * Obs->ddSatObs[i].ddL[0] - FG3_BDS * Obs->ddSatObs[i].ddL[1]) / (FG1_BDS - FG3_BDS) -
				(FG1_BDS * Obs->ddSatObs[i].ddP[0] + FG3_BDS * Obs->ddSatObs[i].ddP[1]) / (FG1_BDS + FG3_BDS);
			comobs[i].PIF = (FG1_BDS * FG1_BDS * Obs->ddSatObs[i].ddP[0] - FG3_BDS * FG3_BDS * Obs->ddSatObs[i].ddP[1]) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
		}

		comobs[i].n = 1;

		// 从上个历元的MWGF数据中查找该卫星的GF和MW组合值
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (Obs->ddCObs[j].Sys == comobs[i].Sys && Obs->ddCObs[j].Prn == comobs[i].Prn)
			{
				// 找到了则比较二者是否在限差内
				double dGF = fabs(comobs[i].GF - Obs->ddCObs[j].GF);
				double dMW = fabs(comobs[i].MW - Obs->ddCObs[j].MW);

				// 没有超限，则标记true并计算MW的平滑值
				if (dGF < 0.05 && dMW < 3)
				{
					Obs->ddSatObs[i].Valid = true;
					comobs[i].MW = (Obs->ddCObs[j].MW * Obs->ddCObs[j].n + comobs[i].MW) / (Obs->ddCObs[j].n + 1);
					comobs[i].n = Obs->ddCObs[j].n + 1;
				}
				break;
				// 超限，则标记为粗差（而初始化本就是false，故不需处理）
			}
			else continue;
		}
		// 如果在上个历元中没有找到，则不知其是否可用，默认为初始化时的false
	}
	// 将缓冲区组合观测值的拷贝到obs里
	memcpy(Obs->ddCObs, comobs, sizeof(comobs));
}
/****************************************************************************
  DetectHalf

  目的：利用前后时刻双差观测值检测半周


  编号：608

  参数:
  Obs			前一时刻双差观测值数据
  CurObs		当前时刻双差观测值数据

****************************************************************************/
//void DetectHalf(const DDCOBS* Obs, DDCOBS* CurObs)
//{
//	for (int i = 0; i < CurObs->Sats; i++)
//	{
//		GNSSSys sys = CurObs->ddSatObs[i].System;
//		int prn = CurObs->ddSatObs[i].Prn;
//		//从上个历元的每颗卫星的观测数据中查找该卫星值
//		int find = 0;
//		int index = 0;
//		for (int j = 0; j < MAXCHANNUM; j++)
//		{
//			if (sys == Obs->ddSatObs[j].System && prn == Obs->ddSatObs[j].Prn)
//			{
//				find = 1;
//				index = j;
//				break;
//			}
//		}
//		for (int flag = 0; flag < 2; flag++)
//		{
//			//检查该卫星的双频相位数据是否有效和完整，若不全或为0，continue
//			if (Obs->ddSatObs[index].ddL[flag] != 0 && CurObs->ddSatObs[i].ddL[flag] != 0)
//			{
//				//比较当前历元该卫星Locktime与上一历元对应Locktime的大小,判断历元间相位数据的连续性
//				if (find)
//				{
//					if (fabs(Obs->ddSatObs[index].ddL[flag]- CurObs->ddSatObs[i].ddL[flag])>0.19)
//					{
//						CurObs->ddSatObs[i].Valid = false;
//						/*printf("cycle slip detected sat:%d\n", CurObs->SatObs[i].Prn);*/
//					}
//				}
//			}
//		}
//	}
//}
/****************************************************************************
  RTKFloat

  目的：双差相对定位浮点解


  编号：606

  输入参数:
  Raw			RTK定位数据，有原始观测值（有卫星位置计算等中间结果）、单差观测值、双差基准星选取结果、星历
  Base			基站SPP定位结果
  Rov			流动站SPP定位结果

  返回值：浮点解解算是否成功，计算成功返回true，少于4颗卫星或者矩阵求逆失败返回false
****************************************************************************/
bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov)
{
	#pragma omp parallel for  // 使用OpenMP进行并行计算
	//观测值
	DetectLockTimeHalf(&Raw->BBasEpk, &Raw->BasEpk); DetectLockTimeHalf(&Raw->BRovEpk, &Raw->RovEpk);//利用接收机的质量数据Locktime检验周跳情况
    //单差观测值
	FormSDEpochObs(&Raw->RovEpk, &Raw->BasEpk, &Raw->SdObs);//站间单差观测值计算
	DetectCycleSlip(&Raw->SdObs);//对站间单差观测值，用MW和GF组合探测周跳
	DetRefSat(&Raw->RovEpk, &Raw->BasEpk, &Raw->SdObs, &Raw->DDObs);//选取基准星
	if (Raw->DDObs.RefPos[0] < 0 && Raw->DDObs.RefPos[1] < 0)//参考星选取失败
	{
		Raw->DDObs.IsSuccess = false;
		return false;
	}
	//双差观测值
	FormDDObs(&Raw->SdObs, &Raw->DDObs);
	DDetectCycleSlip(&Raw->DDObs);
	/*DetectHalf(&Raw->BDDObs, &Raw->DDObs);*/
	//设置基站和流动站位置初值
	double BasPos[3] = { 0 };
	double RovPos[3] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		BasPos[i] = Raw->BasEpk.Pos[i];
		RovPos[i] = Rov->Position[i];
	}
	//计算GPS和BDS双差卫星数
	int validsat = Raw->DDObs.Sats;//观测双差卫星数
	double gpscount = Raw->DDObs.DDSatNum[0];//观测GPS双差卫星数
	double bdscount = Raw->DDObs.DDSatNum[1];//观测BDS双差卫星数
	int sys = 0;
	for (int i0 = 0; i0 < Raw->DDObs.Sats; i0++)
	{
		if (Raw->DDObs.ddSatObs[i0].System == GPS) sys = 0;
		else if (Raw->DDObs.ddSatObs[i0].System == BDS) sys = 1;
		if (Raw->DDObs.ddSatObs[i0].Valid != true)
		{
			if (sys == 0)gpscount--; else bdscount--;
			validsat--;
		}
	}
	//计算基站坐标到所有单差卫星的几何距离
	double BasSat[MAXCHANNUM] = { 0 };
	for (int i = 0; i < Raw->SdObs.SatNum; i++)
	{
		int BasIndex = Raw->SdObs.SdSatObs[i].nBas;
		if (Raw->SdObs.SdSatObs[i].Valid == true)//单差数据有效并卫星位置计算成功
		{
			double dPos[3] = { 0 };//每个观测卫星到基站的距离
			MatrixSubtraction(3, 1, Raw->BasEpk.SatPVT[BasIndex].SatPos, BasPos, dPos);
			BasSat[i] = Norm(3, dPos);
		}
	}
	//确定参数个数
	int parnum = 3 + 2 * validsat;
	double* Qxx = new double[parnum * parnum]();//NBB的逆
	double* Qnn = new double[(2 * validsat) * (2 * validsat)]();//双差浮点解模糊度协因数矩阵
	double* N = new double[validsat * 2]();///双差浮点解模糊度，周
	double sigma0 = 0;//验后单位权中误差
	double pdop = 0;//PDOP值，空间位置精度因子
	int Iterator = 0;  /*计算迭代次数，小于10次*/
	double dRcv[3] = { 0 };//每次迭代的位置增量
	int i = 0;//实际行数
	//N赋初值
	for (int i0 = 0; i0 < Raw->DDObs.Sats; i0++)
	{
		if (Raw->DDObs.ddSatObs[i0].System == GPS) sys = 0;
		else if (Raw->DDObs.ddSatObs[i0].System == BDS) sys = 1;

		if (Raw->DDObs.ddSatObs[i0].Valid != true) continue;
		double WL1=0, WL2 = 0;
		if (sys == 0)
		{
			WL1 = WL1_GPS;  WL2 = WL2_GPS;
		}
		else
		{
			WL1 = WL1_BDS;  WL2 = WL3_BDS;
		}
		N[i * 2] = (Raw->DDObs.ddSatObs[i0].ddL[0] - Raw->DDObs.ddSatObs[i0].ddP[0])/ WL1;
		N[i * 2+1] = (Raw->DDObs.ddSatObs[i0].ddL[1] - Raw->DDObs.ddSatObs[i0].ddP[1]) / WL2;
		i++;
	}
	do {
		//计算流动站到参考星的几何距离
		double RovRefSat[2] = { 0 };
		for (int i = 0; i < 2; i++)
		{
			if (Raw->DDObs.RefPos[i] >= 0)//参考星选取成功
			{
				int RovIndex = Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].nRov;
				double dPos[3] = { 0 };//每个观测卫星到基站的距离
				MatrixSubtraction(3, 1, Raw->RovEpk.SatPVT[RovIndex].SatPos, RovPos, dPos);
				RovRefSat[i] = Norm(3, dPos);
			}
		}
		//计算权矩阵;线性化双差观测方程得到B矩阵和W向量,参考星不用计算
		double* B = new double[(validsat * 4) * parnum]();
		double* w = new double[(validsat * 4)]();
		double* P = new double[(validsat * 4) * (validsat * 4)]();
		int i = 0;//实际矩阵中行数
		for (int i0 = 0; i0 < Raw->DDObs.Sats; i0++)//对双差观测值进行循环
		{
			if (Raw->DDObs.ddSatObs[i0].System == GPS) sys = 0;
			else if (Raw->DDObs.ddSatObs[i0].System == BDS) sys = 1;

			if (Raw->DDObs.ddSatObs[i0].Valid != true) continue;
			int di = Raw->DDObs.ddSatObs[i0].Pos;//当前双差卫星在单差卫星中的位置
			int RovIndex = Raw->SdObs.SdSatObs[di].nRov;//当前单差卫星在流动站观测中的位置
			int RovRefIndex = Raw->DDObs.RefPos[sys];//参考卫星在单差中的序号
			double dPos[3] = { 0 };//每个双差观测卫星到流动站的距离
			MatrixSubtraction(3, 1, Raw->RovEpk.SatPVT[RovIndex].SatPos, RovPos, dPos);
			double d= Norm(3, dPos);
			double l = (RovPos[0] - Raw->RovEpk.SatPVT[RovIndex].SatPos[0]) / d - (RovPos[0] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[RovRefIndex].nRov].SatPos[0]) / RovRefSat[sys];
			double m = (RovPos[1] - Raw->RovEpk.SatPVT[RovIndex].SatPos[1]) / d - (RovPos[1] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[RovRefIndex].nRov].SatPos[1]) / RovRefSat[sys];
			double n = (RovPos[2] - Raw->RovEpk.SatPVT[RovIndex].SatPos[2]) / d - (RovPos[2] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[RovRefIndex].nRov].SatPos[2]) / RovRefSat[sys];
			double rou = d - RovRefSat[sys] - BasSat[di] + BasSat[RovRefIndex];
			for (int j = 0; j < 4; j++)
			{
				B[(i * 4 + j) * (3 + 2 * validsat)] = l;
				B[(i * 4 + j) * (3 + 2 * validsat) + 1] = m;
				B[(i * 4 + j) * (3 + 2 * validsat) + 2] = n;
				if (j % 4 == 0 || j % 4 == 1)
				{
					w[(i * 4 + j)] = Raw->DDObs.ddSatObs[i0].ddP[j] - rou;
					if (sys == 0)
					{
						for (int k = 0; k < gpscount; k++)
						{
							if (k == i)
							{
								P[(i * 4 + j) * (validsat * 4 + 1)] = gpscount / (gpscount + 1);
							}
							else P[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = -1 / (gpscount + 1);
						}
					}
					else
					{
						for (int k = gpscount; k < gpscount + bdscount; k++)
						{
							if (k == i)
							{
								P[(i * 4 + j) * (validsat * 4 + 1)] = bdscount / (bdscount + 1);
							}
							else P[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = -1 / (bdscount + 1);
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
							w[(i * 4 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j-2] - rou - WL1_GPS * N[i * 2];
						}
						if (j % 4 == 3)
						{
							B[(i * 4 + j) * (3 + 2 * validsat) + 2 + i * 2 + 2] = WL2_GPS;
							w[(i * 4 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j - 2] - rou - WL2_GPS * N[i * 2 + 1];
						}
						for (int k = 0; k < gpscount; k++)
						{
							if (k == i)
							{
								P[(i * 4 + j) * (validsat * 4 + 1)] = 1000 * gpscount / (gpscount + 1);
							}
							else P[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = -1000 / (gpscount + 1);
						}
					}
					else
					{
						if (j % 4 == 2 || j % 4 == 3)
						{
							if (j % 4 == 2)
							{
								B[(i * 4 + j) * (3 + 2 * validsat) + 2 + i * 2 + 1] = WL1_BDS;
								w[(i * 4 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j - 2] - rou - WL1_BDS * N[i * 2];
							}
							if (j % 4 == 3)
							{
								B[(i * 4 + j) * (3 + 2 * validsat) + 2 + i * 2 + 2] = WL3_BDS;
								w[(i * 4 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j - 2] - rou - WL3_BDS * N[i * 2 + 1];
							}
							for (int k = gpscount; k < gpscount + bdscount; k++)
							{
								if (k == i)
								{
									P[(i * 4 + j) * (validsat * 4 + 1)] = 1000 * bdscount / (bdscount + 1);
								}
								else P[(i * 4 + j) * (validsat * 4 + 1) + 4 * (k - i)] = -1000 / (bdscount + 1);
							}
						}
					}
				}	
			}
			i++;
		}
		//打印B矩阵和W向量,权矩阵
		/*outputMatrixToFile(B, (validsat * 4), (3 + 2 * validsat), "outputB.txt");
		outputMatrixToFile(w, (validsat * 4), 1, "outputw.txt");
		outputMatrixToFile(P, (validsat * 4), (validsat * 4), "outputP.txt");*/
		//双差观测方程数量大于未知数，则求解
		if (validsat<4)
		{
			Raw->DDObs.IsSuccess = false;
			return false;
		}
		double* BT = new double[parnum * (validsat * 4)]();
		double* NBB = new double[parnum * parnum]();
		double* W = new double[parnum * 1]();
		double* x = new double[parnum]();

		double* Bx = new double[(validsat * 4) * 1]();
		double* v = new double[(validsat * 4) * 1]();
		double sigma = 0;
		//最小二乘解算
		 
		 MatrixTranspose((validsat * 4), parnum, B, BT);
		 MatrixMultiply_ATPPA((validsat * 4), parnum, (validsat * 4), B, P, NBB);
		 /*outputMatrixToFile(NBB, parnum, parnum, "outputNBB.txt");*/
		 MatrixMultiply_APPB(parnum, (validsat * 4), (validsat * 4), 1, BT, P, w, W);
		 if (!lu_decomposition_inverse(parnum, NBB, Qxx))
		 {
			 Raw->DDObs.IsSuccess = false;
			 return false;
		 }
		 MatrixMultiply(parnum, parnum, parnum, 1, Qxx, W, x);
		 //定位精度评价
		 MatrixMultiply((validsat * 4), parnum, parnum, 1, B, x, Bx);
		 MatrixSubtraction((validsat * 4), 1, Bx, w, v);
		 MatrixMultiply_ATPPA((validsat * 4), 1, (validsat * 4), v, P, &sigma);
		 sigma0 = sqrt(sigma / (validsat * 4 - parnum));
		 pdop = sqrt(Qxx[0] + Qxx[parnum+1] + Qxx[2* parnum + 2]);
		//更新流动站的位置和双差模糊度参数，为下一轮循环做准备
		 dRcv[0] = x[0];
		 dRcv[1] = x[1];
		 dRcv[2] = x[2];
		 if (bdscount == 0)
		 {
			 for (i = 0; i < 2 * gpscount; i++)
			 {
				 N[i] += x[3 + i];
			 }
		 }
		 else if (gpscount == 0)
		 {
			 for (i = 0; i < 2*bdscount; i++)
			 {
				 N[i] += x[3 + i];
			 }
		 }
		 else
		 {
			 for (i = 0; i < 2 * (gpscount+bdscount); i++)
			 {
				 N[i] += x[3 + i];
			 }
		 }
		 MatrixAddition2(3, 1, dRcv, RovPos);
		Iterator++;
		//动态数组释放
		delete[]B;
		delete[]w;
		delete[]P;
		delete[]BT;
		delete[]NBB;
		delete[]W;
		delete[]x;
		delete[]Bx;
		delete[]v;
	}while(Norm(3, dRcv) > 1e-8 && Iterator < 10);
	//Raw->DDObs.Sats = validsat;//有效观测双差卫星数
	//Raw->DDObs.DDSatNum[0] = gpscount;//有效观测GPS双差卫星数
	//Raw->DDObs.DDSatNum[1] = bdscount;//有效观测BDS双差卫星数
	//保存双差浮点解模糊度及其协因数矩阵，用于LAMBDA模糊度固定
	for (int i = 0; i < 2 * validsat; i++) {
		for (int j = 0; j < 2 * validsat; j++) {
			Qnn[i * (2 * validsat) + j] = Qxx[(parnum - 2 * validsat + i) * parnum + (parnum - 2 * validsat + j)];
		}
	}
	//保存浮点解基线向量，精度指标
	MatrixSubtraction(3, 1, RovPos, BasPos, Raw->DDObs.dPos);// 基线向量
	memcpy(Raw->DDObs.RovXYZ, RovPos, 3 * sizeof(double));
	XYZToBLH(RovPos, Raw->DDObs.RovBLH, R_WGS84, F_WGS84);
	Comp_dEnu(BasPos, RovPos, Raw->DDObs.dENH);
	Raw->DDObs.FixRMS[0] = sigma0 * pdop;// 浮点解定位中rms误差
	Raw->DDObs.IsSuccess = true;
	//LAMBDA模糊度固定
	if (lambda(validsat * 2, 2, N, Qnn, Raw->DDObs.FixedAmb, Raw->DDObs.ResAmb) != 0)
	{
		Raw->DDObs.bFixed = false;
		Raw->DDObs.Ratio = 0;
		return false;
	}
	else
	{
		Raw->DDObs.Ratio = Raw->DDObs.ResAmb[1] / Raw->DDObs.ResAmb[0];//Ratio值
	}
	if (Raw->DDObs.Ratio < 1.8)
	{
		Raw->DDObs.bFixed = false;
	}
	else Raw->DDObs.bFixed = true;
	delete[]N;
	delete[]Qxx;
	delete[]Qnn;
	if (Raw->DDObs.bFixed == true) RTKFixed(Raw, RovPos,BasSat);//如果固定成功，更新基线向量和固定解定位中rms误差
	return true;
}
void RTKFixed(RAWDAT* Raw, double * RovPos,double *BasSat)
{
	//设置基站和流动站位置初值
	double BasPos[3] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		BasPos[i] = Raw->BasEpk.Pos[i];
	}
	//计算GPS和BDS双差卫星数
	int validsat = Raw->DDObs.Sats;//观测双差卫星数
	double gpscount = Raw->DDObs.DDSatNum[0];//观测GPS双差卫星数
	double bdscount = Raw->DDObs.DDSatNum[1];//观测BDS双差卫星数
	int sys = 0;
	for (int i0 = 0; i0 < Raw->DDObs.Sats; i0++)
	{
		if (Raw->DDObs.ddSatObs[i0].System == GPS) sys = 0;
		else if (Raw->DDObs.ddSatObs[i0].System == BDS) sys = 1;
		if (Raw->DDObs.ddSatObs[i0].Valid != true)
		{
			if (sys == 0)gpscount--; else bdscount--;
			validsat--;
		}
	}
	double dRcv[3] = { 0 };//位置增量
	double sigma0 = 0;//验后单位权中误差
	double pdop = 0;//PDOP值，空间位置精度因子
	int Iterator = 0;  /*计算迭代次数，小于10次*/
	do {
		//计算流动站到参考星的几何距离
		double RovRefSat[2] = { 0 };
		for (int i = 0; i < 2; i++)
		{
			if (Raw->DDObs.RefPos[i] >= 0)//参考星选取成功
			{
				int RovIndex = Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].nRov;
				double dPos[3] = { 0 };//每个观测卫星到基站的距离
				MatrixSubtraction(3, 1, Raw->RovEpk.SatPVT[RovIndex].SatPos, RovPos, dPos);
				RovRefSat[i] = Norm(3, dPos);
			}
		}
		//线性化双差观测方程得到B矩阵和W向量,参考星不用计算
		double* B = new double[(validsat * 2) * 3]();
		double* w = new double[(validsat * 2)]();
		double* P = new double[(validsat * 2) * (validsat * 2)]();
		int i = 0;//实际矩阵中行数
		int sys = 0;//卫星序号
		for (int i0 = 0; i0 < Raw->DDObs.Sats; i0++)//对双差观测值进行循环
		{
			if (Raw->DDObs.ddSatObs[i0].System == GPS) sys = 0;
			else if (Raw->DDObs.ddSatObs[i0].System == BDS) sys = 1;

			if (Raw->DDObs.ddSatObs[i0].Valid != true) continue;
			int di = Raw->DDObs.ddSatObs[i0].Pos;//当前双差卫星在单差卫星中的位置
			int RovIndex = Raw->SdObs.SdSatObs[di].nRov;//当前单差卫星在流动站观测中的位置
			int RovRefIndex = Raw->DDObs.RefPos[sys];//参考卫星在单差中的序号
			double dPos[3] = { 0 };//每个双差观测卫星到流动站的距离
			MatrixSubtraction(3, 1, Raw->RovEpk.SatPVT[RovIndex].SatPos, RovPos, dPos);
			double d = Norm(3, dPos);
			double l = (RovPos[0] - Raw->RovEpk.SatPVT[RovIndex].SatPos[0]) / d - (RovPos[0] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[RovRefIndex].nRov].SatPos[0]) / RovRefSat[sys];
			double m = (RovPos[1] - Raw->RovEpk.SatPVT[RovIndex].SatPos[1]) / d - (RovPos[1] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[RovRefIndex].nRov].SatPos[1]) / RovRefSat[sys];
			double n = (RovPos[2] - Raw->RovEpk.SatPVT[RovIndex].SatPos[2]) / d - (RovPos[2] - Raw->RovEpk.SatPVT[Raw->SdObs.SdSatObs[RovRefIndex].nRov].SatPos[2]) / RovRefSat[sys];
			double rou = d - RovRefSat[sys] - BasSat[di] + BasSat[RovRefIndex];
			for (int j = 0; j < 2; j++)
			{
				B[(i * 2 + j) * 3] = l;
				B[(i * 2 + j) * 3 + 1] = m;
				B[(i * 2 + j) * 3 + 2] = n;
				if (sys == 0)
				{
					if (j % 2 == 0)
					{
						w[(i * 2 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j] - rou - WL1_GPS * Raw->DDObs.FixedAmb[i * 2];
					}
					else w[(i * 2 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j] - rou - WL2_GPS * Raw->DDObs.FixedAmb[i * 2 + 1];
					for (int k = 0; k < gpscount; k++)
					{
						if (k == i)
						{
							P[(i * 2 + j) * (validsat * 2 + 1)] = gpscount / (gpscount + 1);
						}
						else P[(i * 2 + j) * (validsat * 2 + 1) + 2 * (k - i)] = -1 / (gpscount + 1);
					}
				}
				else
				{
					if (j % 2 == 0)
					{
						w[(i * 2 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j] - rou - WL1_BDS * Raw->DDObs.FixedAmb[i * 2];
					}
					else w[(i * 2 + j)] = Raw->DDObs.ddSatObs[i0].ddL[j] - rou - WL3_BDS * Raw->DDObs.FixedAmb[i * 2 + 1];
					for (int k = gpscount; k < gpscount + bdscount; k++)
					{
						if (k == i)
						{
							P[(i * 2 + j) * (validsat * 2 + 1)] = bdscount / (bdscount + 1);
						}
						else P[(i * 2 + j) * (validsat * 2 + 1) + 2 * (k - i)] = -1 / (bdscount + 1);
					}
				}
			}
			i++;
		}
		//outputMatrixToFile(B, (validsat * 2), 3, "outputB.txt");
		//outputMatrixToFile(w, (validsat * 2), 1, "outputw.txt");
		//outputMatrixToFile(P, (validsat * 2), (validsat * 2), "outputP.txt");
		double* BT = new double[3 * (validsat * 2)]();
		double NBB[3 * 3] = { 0 };
		double Qxx[3 * 3] = { 0 };
		double W[3 * 1] = { 0 };
		double x[3 * 1] = { 0 };

		double* Bx = new double[(validsat * 2) * 1]();
		double* v = new double[(validsat * 2) * 1]();
		double sigma = 0;
		//最小二乘解算
		MatrixTranspose((validsat * 2), 3, B, BT);
		MatrixMultiply_ATPPA((validsat * 2), 3,(validsat * 2), B, P, NBB);
		MatrixMultiply_APPB(3, (validsat * 2), (validsat * 2), 1, BT, P, w, W);
		lu_decomposition_inverse(3, NBB, Qxx);
		MatrixMultiply(3, 3, 3, 1, Qxx, W, x);
		//定位精度评价
		MatrixMultiply((validsat * 2), 3, 3, 1, B, x, Bx);
		MatrixSubtraction((validsat * 2), 1, Bx, w, v);
		MatrixMultiply_ATPPA((validsat * 2), 1, (validsat * 2), v, P, &sigma);
		sigma0 = sqrt(sigma / (validsat * 2 - 3));
		pdop = sqrt(Qxx[0] + Qxx[4] + Qxx[8]);
		//更新流动站的位置和双差模糊度参数，为下一轮循环做准备
		dRcv[0] = x[0];
		dRcv[1] = x[1];
		dRcv[2] = x[2];
		MatrixAddition2(3, 1, dRcv, RovPos);
		Iterator++;
		//动态数组释放
		delete[]B;
		delete[]w;
		delete[]P;
		delete[]BT;
		delete[]Bx;
		delete[]v;
	} while (Norm(3, dRcv) > 1e-8 && Iterator < 10);
	//Raw->DDObs.Sats = validsat;//有效观测双差卫星数
	//Raw->DDObs.DDSatNum[0] = gpscount;//有效观测GPS双差卫星数
	//Raw->DDObs.DDSatNum[1] = bdscount;//有效观测BDS双差卫星数
	MatrixSubtraction(3, 1, RovPos, BasPos, Raw->DDObs.dPos);// 基线向量
	memcpy(Raw->DDObs.RovXYZ, RovPos, 3 * sizeof(double));
	XYZToBLH(RovPos, Raw->DDObs.RovBLH, R_WGS84, F_WGS84);
	Comp_dEnu(BasPos, RovPos, Raw->DDObs.dENH);
	Raw->DDObs.FixRMS[0] = sigma0 * pdop;
}