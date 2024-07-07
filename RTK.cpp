// RTK.cpp : ���ļ�����RTK�������̶�����㡣
//
#include "RTK_Structs.h"
#define limt 0.1
#define limT 3

/****************************************************************************
  GetSynObs

  Ŀ�ģ����ݽ�����ʱ��ͬ��������RTK��λ������ԭʼ�۲����ݺ�����


  ��ţ�601

  ����:
  FBas			��վ�ļ�ָ��
  FRov			�ƶ�վ�ļ�ָ��
  BasSock		��վ����˿�
  RovSock		�ƶ�վ����˿�
  Raw			RTK��λ����

  ����ֵ��ͬ���ɹ�����1��ͬ��ʧ�ܷ���0
****************************************************************************/
int GetSynObs(FILE* FBas, FILE* FRov, SOCKET& BasSock,SOCKET& RovSock, RAWDAT* Raw)
{
	Raw->BBasEpk = Raw->BasEpk; Raw->BRovEpk = Raw->RovEpk; Raw->BDDObs = Raw->DDObs; //��ͬ��ǰ����һ��ԭʼ�۲�ֵ����
	if (FBas!=NULL&&(!(feof(FBas) || feof(FRov))))
	{
		static unsigned char Basbuff[MAXRAWLEN], Rovbuff[MAXRAWLEN];
		static int Blen = 0, Rlen = 0;

		int dt = 0;
		int Rresult = 0, Bresult = 0;
		while (Rresult != 1) {
			//������һ��ѭ����buff��ʣ��ֵ
			int Rtemp = Rlen;
			// ���ļ��ж�ȡ���ݵ�buff
			Rlen = fread(Rovbuff + Rtemp, sizeof(unsigned char), MAXRAWLEN - Rtemp, FRov);
			Rlen += Rtemp;
			// ��������
			Rresult = DecodeNovOem7Dat(Rovbuff, Rlen, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph,1);
		}
		do {
			if (dt < -limt) {
				Rresult = 0;
				while (Rresult != 1) {
					//������һ��ѭ����buff��ʣ��ֵ
					int Rtemp = Rlen;
					// ���ļ��ж�ȡ���ݵ�buff
					Rlen = fread(Rovbuff + Rtemp, sizeof(unsigned char), MAXRAWLEN - Rtemp, FRov);
					Rlen += Rtemp;
					// ��������
					Rresult = DecodeNovOem7Dat(Rovbuff, Rlen, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph,1);
				}
			}
			dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
			if (fabs(dt) <= limt)	break;
			if (dt > limt) {
				Bresult = 0;
				while (Bresult != 1) {
					//������һ��ѭ����buff��ʣ��ֵ
					int Btemp = Blen;
					// ���ļ��ж�ȡ���ݵ�buff
					Blen = fread(Basbuff + Btemp, sizeof(unsigned char), MAXRAWLEN - Btemp, FBas);
					Blen += Btemp;
					// ��������
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
		//�洢����
		static unsigned char BasBuff[3 * MAXRAWLEN], RovBuff[3 * MAXRAWLEN];
		static int BlenD = 0, RlenD = 0;
		unsigned char Basbuff[MAXRAWLEN], Rovbuff[MAXRAWLEN];
		int BlenR = 0, RlenR = 0;
		//ѭ����־
		int dt = 0;
		int Rresult = 0, Bresult = 0;
		Sleep(980);
		if ((RlenR = recv(RovSock, (char*)Rovbuff, MAXRAWLEN, 0)) > 0) {
			/*printf("%d\n", RlenR);*/
			memcpy(RovBuff + RlenD, Rovbuff, RlenR);
			RlenD += RlenR;
			memset(Rovbuff, 0, MAXRAWLEN);
			//while (Rresult != 5) {
			//	// ��������
			//	Rresult = DecodeNovOem7Dat(RovBuff, RlenD, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph,0);
			//}
			Rresult = DecodeNovOem7Dat(RovBuff, RlenD, &(Raw->RovEpk), Raw->GpsEph, Raw->BdsEph, 0);
		}
		if ((BlenR = recv(BasSock, (char*)Basbuff, MAXRAWLEN, 0)) > 0) {
			memcpy(BasBuff + BlenD, Basbuff, BlenR);
			BlenD += BlenR;
			memset(Basbuff, 0, MAXRAWLEN);
			//while (Bresult != 5) {
			//	// ��������
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
		//			// ��������
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
		//					// ��������
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
		//					// ��������
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

  Ŀ�ģ����ý��ջ�����������Locktime��Parity�����������Ͱ������,������ְ��ܻ����������۲�����Valid=false


  ��ţ�602

  ����:
  Obs			ǰһʱ�̹۲�ֵ����
  CurObs		��ǰʱ�̹۲�ֵ����

****************************************************************************/
void DetectLockTimeHalf(const EPOCHOBS* Obs, EPOCHOBS* CurObs)
{
	int satnum = CurObs->SatNum;
	for (int i = 0; i < satnum; i++)
	{
		GNSSSys sys = CurObs->SatObs[i].System;
		int prn = CurObs->SatObs[i].Prn;
		//���ϸ���Ԫ��ÿ�����ǵĹ۲������в��Ҹ�����ֵ
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
			//��鵱ǰʱ�̸����ǵ�˫Ƶ��λ�����Ƿ���Ч������
			if (CurObs->SatObs[i].L[flag] != 0)
			{
				//�ж���λ�����Ƿ���ڰ���
				if (CurObs->SatObs[i].half[flag] == 0)
				{
					CurObs->SatObs[i].Valid = false;
					/*printf("half cycle detected sat:%d\n", CurObs->SatObs[i].Prn);*/
				}
			}
			//�������ǵ�˫Ƶ��λ�����Ƿ���Ч������������ȫ��Ϊ0��continue
			if (Obs->SatObs[index].L[flag] != 0 && CurObs->SatObs[i].L[flag] != 0)
			{
				//�Ƚϵ�ǰ��Ԫ������Locktime����һ��Ԫ��ӦLocktime�Ĵ�С,�ж���Ԫ����λ���ݵ�������
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

  Ŀ�ģ�վ�䵥��۲�ֵ����


  ��ţ�603

  ����:
  EpkA			�ƶ�վ�۲�����
  EpkB			��վ�۲�����
  SDObs			����۲�����

****************************************************************************/
void FormSDEpochObs(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs)
{
	memset(&SDObs->SdSatObs, 0, sizeof(SDObs->SdSatObs));// ʹ��memset()��յ���ṹ���еĹ۲�����
	//����ͬ���ǡ���ͬƵ�ʡ���ͬ���͹۲�ֵ����վ�����
	SDObs->Time = EpkA->Time;
	unsigned long satsum = EpkA->SatNum;
	unsigned short satcount = 0;//�ƶ�վ���Ǽ���
	unsigned short n = 0;//�������Ǽ���
	while (satcount < satsum)
	{
		GNSSSys sys = EpkA->SatObs[satcount].System;
		unsigned short prn = EpkA->SatObs[satcount].Prn;//���Ǻź�ϵͳ
		//����վ�۲�����Ǻź�ϵͳ���ڻ�վ�۲⵽���������Ƿ���ڣ����ھͷ��ص�ǰ�������±�
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
			//�������ǵ��ƶ�վ�ͻ�վ�����Ƿ���Ч������������ȫ��Ϊ0��continue
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

  Ŀ�ģ���վ�䵥��۲�ֵ����MW��GF���̽������


  ��ţ�604

  ����:
  Obs			����۲�����
****************************************************************************/
void DetectCycleSlip(SDEPOCHOBS* Obs)
{
	// Ϊ�˲��ƻ���һ��Ԫ����Ϲ۲�ֵ����������Ļ�����
	MWGF comobs[MAXCHANNUM];// ��ŵ�ǰ��Ԫ�������ֵ
	for (int i = 0; i < Obs->SatNum; i++)
	{
		// ���ÿһ�����ǵ�˫Ƶα�����λ�����Ƿ�����
		if (fabs(Obs->SdSatObs[i].dP[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[1]) < 1e-8 || fabs(Obs->SdSatObs[i].dL[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dL[1]) < 1e-8)
		{
			Obs->SdSatObs[i].Valid = false;
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

		// ���ϸ���Ԫ��MWGF�����в��Ҹ����ǵ�GF��MW���ֵ
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (Obs->SdCObs[j].Sys == comobs[i].Sys && Obs->SdCObs[j].Prn == comobs[i].Prn)
			{
				// �ҵ�����Ƚ϶����Ƿ����޲���
				double dGF = fabs(comobs[i].GF - Obs->SdCObs[j].GF);
				double dMW = fabs(comobs[i].MW - Obs->SdCObs[j].MW);

				// û�г��ޣ�����true������MW��ƽ��ֵ
				if (dGF < 0.05 && dMW < 3)
				{
					Obs->SdSatObs[i].Valid = true;
					comobs[i].MW = (Obs->SdCObs[j].MW * Obs->SdCObs[j].n + comobs[i].MW) / (Obs->SdCObs[j].n + 1);
					comobs[i].n = Obs->SdCObs[j].n + 1;
				}
				break;
				// ���ޣ�����Ϊ�ֲ����ʼ��������false���ʲ��账��
			}
			else continue;
		}
		// ������ϸ���Ԫ��û���ҵ�����֪���Ƿ���ã�Ĭ��Ϊ��ʼ��ʱ��false
	}
	// ����������Ϲ۲�ֵ�Ŀ�����obs��
	memcpy(Obs->SdCObs, comobs, sizeof(comobs));
}
/****************************************************************************
  DetRefSat

  Ŀ�ģ�ѡȡ��׼��


  ��ţ�605

  ����:
  EpkA			�ƶ�վ�۲�����
  EpkB			��վ�۲�����
  SDObs			����۲�����
  DDObs			˫����ص�����
****************************************************************************/
void DetRefSat(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
	//�Ը����ǵ���ϵͳ���ֱ�ѡȡһ�Ź۲�ֵ���������ߵ�������Ϊ�ο��ǣ��������ǵĵ���۲�ֵ��òο��Ƕ�Ӧ�ĵ���۲�ֵ���
	//�ο���ѡȡҪ��:α����ز���λͨ������̽�⣬û�дֲ���������;������������������λ�ü���ɹ�;�߶ȽǴ��CN0��
	int RefPrn[2] = { -1, -1 }, RefPos[2] = { -1, -1 };         // �ο������Ǻ��뵥��۲��еĴ洢λ�ã�0=GPS; 1=BDS
	double MAXElev[2] = { 0,0 };	  //������Ǹ߶Ƚǣ�0=GPS; 1=BDS
	//��ÿ����������
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		int RovIndex = SDObs->SdSatObs[i].nRov; int BasIndex = SDObs->SdSatObs[i].nBas;
		//����������Ч������λ�ü���ɹ�
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

  Ŀ�ģ��Ǽ�˫��۲�ֵ����


  ��ţ�606

  ����:
  SDObs			����۲�����
  DDObs			˫���������

****************************************************************************/
void FormDDObs(const SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
	memset(&DDObs->ddSatObs, 0, sizeof(DDObs->ddSatObs));// ʹ��memset()��յ���ṹ���еĹ۲�����
	//����ͬ���ǡ���ͬƵ�ʡ���ͬ���͹۲�ֵ�����Ǽ����
	DDObs->Time = SDObs->Time;
	//����GPS��BDS˫��������
	int validsat = 0;//�۲�˫��������
	int gpscount = 0;//�۲�GPS˫��������
	int bdscount = 0;//�۲�BDS˫��������
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
		int SdRefIndex = DDObs->RefPos[sys];//�ο������ڵ����е����
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

  Ŀ�ģ���վ��˫��۲�ֵ����MW��GF���̽������


  ��ţ�607

  ����:
  Obs			ǰһʱ�̹۲�ֵ����
  CurObs		��ǰʱ�̹۲�ֵ����
****************************************************************************/
void DDetectCycleSlip(DDCOBS* Obs)
{
	// Ϊ�˲��ƻ���һ��Ԫ����Ϲ۲�ֵ����������Ļ�����
	MWGF comobs[MAXCHANNUM];// ��ŵ�ǰ��Ԫ�������ֵ

	for (int i = 0; i < Obs->Sats; i++)
	{
		// ���ÿһ�����ǵ�˫Ƶα�����λ�����Ƿ�����
		if (fabs(Obs->ddSatObs[i].ddP[0]) < 1e-8 || fabs(Obs->ddSatObs[i].ddP[1]) < 1e-8 || fabs(Obs->ddSatObs[i].ddL[0]) < 1e-8 || fabs(Obs->ddSatObs[i].ddL[1]) < 1e-8)
		{
			Obs->ddSatObs[i].Valid = false;
			continue; //�����������ݲ�ȱ����������Ϲ۲�ֵ�ҹ۲�ֵ��Ч������Ϊfalse
		}
		// ���㵱ǰ��Ԫ�ĸ����ǵ�GF��MW���ֵ
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

		// ���ϸ���Ԫ��MWGF�����в��Ҹ����ǵ�GF��MW���ֵ
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (Obs->ddCObs[j].Sys == comobs[i].Sys && Obs->ddCObs[j].Prn == comobs[i].Prn)
			{
				// �ҵ�����Ƚ϶����Ƿ����޲���
				double dGF = fabs(comobs[i].GF - Obs->ddCObs[j].GF);
				double dMW = fabs(comobs[i].MW - Obs->ddCObs[j].MW);

				// û�г��ޣ�����true������MW��ƽ��ֵ
				if (dGF < 0.05 && dMW < 3)
				{
					Obs->ddSatObs[i].Valid = true;
					comobs[i].MW = (Obs->ddCObs[j].MW * Obs->ddCObs[j].n + comobs[i].MW) / (Obs->ddCObs[j].n + 1);
					comobs[i].n = Obs->ddCObs[j].n + 1;
				}
				break;
				// ���ޣ�����Ϊ�ֲ����ʼ��������false���ʲ��账��
			}
			else continue;
		}
		// ������ϸ���Ԫ��û���ҵ�����֪���Ƿ���ã�Ĭ��Ϊ��ʼ��ʱ��false
	}
	// ����������Ϲ۲�ֵ�Ŀ�����obs��
	memcpy(Obs->ddCObs, comobs, sizeof(comobs));
}
/****************************************************************************
  DetectHalf

  Ŀ�ģ�����ǰ��ʱ��˫��۲�ֵ������


  ��ţ�608

  ����:
  Obs			ǰһʱ��˫��۲�ֵ����
  CurObs		��ǰʱ��˫��۲�ֵ����

****************************************************************************/
//void DetectHalf(const DDCOBS* Obs, DDCOBS* CurObs)
//{
//	for (int i = 0; i < CurObs->Sats; i++)
//	{
//		GNSSSys sys = CurObs->ddSatObs[i].System;
//		int prn = CurObs->ddSatObs[i].Prn;
//		//���ϸ���Ԫ��ÿ�����ǵĹ۲������в��Ҹ�����ֵ
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
//			//�������ǵ�˫Ƶ��λ�����Ƿ���Ч������������ȫ��Ϊ0��continue
//			if (Obs->ddSatObs[index].ddL[flag] != 0 && CurObs->ddSatObs[i].ddL[flag] != 0)
//			{
//				//�Ƚϵ�ǰ��Ԫ������Locktime����һ��Ԫ��ӦLocktime�Ĵ�С,�ж���Ԫ����λ���ݵ�������
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

  Ŀ�ģ�˫����Զ�λ�����


  ��ţ�606

  �������:
  Raw			RTK��λ���ݣ���ԭʼ�۲�ֵ��������λ�ü�����м�����������۲�ֵ��˫���׼��ѡȡ���������
  Base			��վSPP��λ���
  Rov			����վSPP��λ���

  ����ֵ�����������Ƿ�ɹ�������ɹ�����true������4�����ǻ��߾�������ʧ�ܷ���false
****************************************************************************/
bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov)
{
	#pragma omp parallel for  // ʹ��OpenMP���в��м���
	//�۲�ֵ
	DetectLockTimeHalf(&Raw->BBasEpk, &Raw->BasEpk); DetectLockTimeHalf(&Raw->BRovEpk, &Raw->RovEpk);//���ý��ջ�����������Locktime�����������
    //����۲�ֵ
	FormSDEpochObs(&Raw->RovEpk, &Raw->BasEpk, &Raw->SdObs);//վ�䵥��۲�ֵ����
	DetectCycleSlip(&Raw->SdObs);//��վ�䵥��۲�ֵ����MW��GF���̽������
	DetRefSat(&Raw->RovEpk, &Raw->BasEpk, &Raw->SdObs, &Raw->DDObs);//ѡȡ��׼��
	if (Raw->DDObs.RefPos[0] < 0 && Raw->DDObs.RefPos[1] < 0)//�ο���ѡȡʧ��
	{
		Raw->DDObs.IsSuccess = false;
		return false;
	}
	//˫��۲�ֵ
	FormDDObs(&Raw->SdObs, &Raw->DDObs);
	DDetectCycleSlip(&Raw->DDObs);
	/*DetectHalf(&Raw->BDDObs, &Raw->DDObs);*/
	//���û�վ������վλ�ó�ֵ
	double BasPos[3] = { 0 };
	double RovPos[3] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		BasPos[i] = Raw->BasEpk.Pos[i];
		RovPos[i] = Rov->Position[i];
	}
	//����GPS��BDS˫��������
	int validsat = Raw->DDObs.Sats;//�۲�˫��������
	double gpscount = Raw->DDObs.DDSatNum[0];//�۲�GPS˫��������
	double bdscount = Raw->DDObs.DDSatNum[1];//�۲�BDS˫��������
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
	//�����վ���굽���е������ǵļ��ξ���
	double BasSat[MAXCHANNUM] = { 0 };
	for (int i = 0; i < Raw->SdObs.SatNum; i++)
	{
		int BasIndex = Raw->SdObs.SdSatObs[i].nBas;
		if (Raw->SdObs.SdSatObs[i].Valid == true)//����������Ч������λ�ü���ɹ�
		{
			double dPos[3] = { 0 };//ÿ���۲����ǵ���վ�ľ���
			MatrixSubtraction(3, 1, Raw->BasEpk.SatPVT[BasIndex].SatPos, BasPos, dPos);
			BasSat[i] = Norm(3, dPos);
		}
	}
	//ȷ����������
	int parnum = 3 + 2 * validsat;
	double* Qxx = new double[parnum * parnum]();//NBB����
	double* Qnn = new double[(2 * validsat) * (2 * validsat)]();//˫����ģ����Э��������
	double* N = new double[validsat * 2]();///˫����ģ���ȣ���
	double sigma0 = 0;//���λȨ�����
	double pdop = 0;//PDOPֵ���ռ�λ�þ�������
	int Iterator = 0;  /*�������������С��10��*/
	double dRcv[3] = { 0 };//ÿ�ε�����λ������
	int i = 0;//ʵ������
	//N����ֵ
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
		//��������վ���ο��ǵļ��ξ���
		double RovRefSat[2] = { 0 };
		for (int i = 0; i < 2; i++)
		{
			if (Raw->DDObs.RefPos[i] >= 0)//�ο���ѡȡ�ɹ�
			{
				int RovIndex = Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].nRov;
				double dPos[3] = { 0 };//ÿ���۲����ǵ���վ�ľ���
				MatrixSubtraction(3, 1, Raw->RovEpk.SatPVT[RovIndex].SatPos, RovPos, dPos);
				RovRefSat[i] = Norm(3, dPos);
			}
		}
		//����Ȩ����;���Ի�˫��۲ⷽ�̵õ�B�����W����,�ο��ǲ��ü���
		double* B = new double[(validsat * 4) * parnum]();
		double* w = new double[(validsat * 4)]();
		double* P = new double[(validsat * 4) * (validsat * 4)]();
		int i = 0;//ʵ�ʾ���������
		for (int i0 = 0; i0 < Raw->DDObs.Sats; i0++)//��˫��۲�ֵ����ѭ��
		{
			if (Raw->DDObs.ddSatObs[i0].System == GPS) sys = 0;
			else if (Raw->DDObs.ddSatObs[i0].System == BDS) sys = 1;

			if (Raw->DDObs.ddSatObs[i0].Valid != true) continue;
			int di = Raw->DDObs.ddSatObs[i0].Pos;//��ǰ˫�������ڵ��������е�λ��
			int RovIndex = Raw->SdObs.SdSatObs[di].nRov;//��ǰ��������������վ�۲��е�λ��
			int RovRefIndex = Raw->DDObs.RefPos[sys];//�ο������ڵ����е����
			double dPos[3] = { 0 };//ÿ��˫��۲����ǵ�����վ�ľ���
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
		//��ӡB�����W����,Ȩ����
		/*outputMatrixToFile(B, (validsat * 4), (3 + 2 * validsat), "outputB.txt");
		outputMatrixToFile(w, (validsat * 4), 1, "outputw.txt");
		outputMatrixToFile(P, (validsat * 4), (validsat * 4), "outputP.txt");*/
		//˫��۲ⷽ����������δ֪���������
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
		//��С���˽���
		 
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
		 //��λ��������
		 MatrixMultiply((validsat * 4), parnum, parnum, 1, B, x, Bx);
		 MatrixSubtraction((validsat * 4), 1, Bx, w, v);
		 MatrixMultiply_ATPPA((validsat * 4), 1, (validsat * 4), v, P, &sigma);
		 sigma0 = sqrt(sigma / (validsat * 4 - parnum));
		 pdop = sqrt(Qxx[0] + Qxx[parnum+1] + Qxx[2* parnum + 2]);
		//��������վ��λ�ú�˫��ģ���Ȳ�����Ϊ��һ��ѭ����׼��
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
		//��̬�����ͷ�
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
	//Raw->DDObs.Sats = validsat;//��Ч�۲�˫��������
	//Raw->DDObs.DDSatNum[0] = gpscount;//��Ч�۲�GPS˫��������
	//Raw->DDObs.DDSatNum[1] = bdscount;//��Ч�۲�BDS˫��������
	//����˫����ģ���ȼ���Э������������LAMBDAģ���ȹ̶�
	for (int i = 0; i < 2 * validsat; i++) {
		for (int j = 0; j < 2 * validsat; j++) {
			Qnn[i * (2 * validsat) + j] = Qxx[(parnum - 2 * validsat + i) * parnum + (parnum - 2 * validsat + j)];
		}
	}
	//���渡����������������ָ��
	MatrixSubtraction(3, 1, RovPos, BasPos, Raw->DDObs.dPos);// ��������
	memcpy(Raw->DDObs.RovXYZ, RovPos, 3 * sizeof(double));
	XYZToBLH(RovPos, Raw->DDObs.RovBLH, R_WGS84, F_WGS84);
	Comp_dEnu(BasPos, RovPos, Raw->DDObs.dENH);
	Raw->DDObs.FixRMS[0] = sigma0 * pdop;// ����ⶨλ��rms���
	Raw->DDObs.IsSuccess = true;
	//LAMBDAģ���ȹ̶�
	if (lambda(validsat * 2, 2, N, Qnn, Raw->DDObs.FixedAmb, Raw->DDObs.ResAmb) != 0)
	{
		Raw->DDObs.bFixed = false;
		Raw->DDObs.Ratio = 0;
		return false;
	}
	else
	{
		Raw->DDObs.Ratio = Raw->DDObs.ResAmb[1] / Raw->DDObs.ResAmb[0];//Ratioֵ
	}
	if (Raw->DDObs.Ratio < 1.8)
	{
		Raw->DDObs.bFixed = false;
	}
	else Raw->DDObs.bFixed = true;
	delete[]N;
	delete[]Qxx;
	delete[]Qnn;
	if (Raw->DDObs.bFixed == true) RTKFixed(Raw, RovPos,BasSat);//����̶��ɹ������»��������͹̶��ⶨλ��rms���
	return true;
}
void RTKFixed(RAWDAT* Raw, double * RovPos,double *BasSat)
{
	//���û�վ������վλ�ó�ֵ
	double BasPos[3] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		BasPos[i] = Raw->BasEpk.Pos[i];
	}
	//����GPS��BDS˫��������
	int validsat = Raw->DDObs.Sats;//�۲�˫��������
	double gpscount = Raw->DDObs.DDSatNum[0];//�۲�GPS˫��������
	double bdscount = Raw->DDObs.DDSatNum[1];//�۲�BDS˫��������
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
	double dRcv[3] = { 0 };//λ������
	double sigma0 = 0;//���λȨ�����
	double pdop = 0;//PDOPֵ���ռ�λ�þ�������
	int Iterator = 0;  /*�������������С��10��*/
	do {
		//��������վ���ο��ǵļ��ξ���
		double RovRefSat[2] = { 0 };
		for (int i = 0; i < 2; i++)
		{
			if (Raw->DDObs.RefPos[i] >= 0)//�ο���ѡȡ�ɹ�
			{
				int RovIndex = Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[i]].nRov;
				double dPos[3] = { 0 };//ÿ���۲����ǵ���վ�ľ���
				MatrixSubtraction(3, 1, Raw->RovEpk.SatPVT[RovIndex].SatPos, RovPos, dPos);
				RovRefSat[i] = Norm(3, dPos);
			}
		}
		//���Ի�˫��۲ⷽ�̵õ�B�����W����,�ο��ǲ��ü���
		double* B = new double[(validsat * 2) * 3]();
		double* w = new double[(validsat * 2)]();
		double* P = new double[(validsat * 2) * (validsat * 2)]();
		int i = 0;//ʵ�ʾ���������
		int sys = 0;//�������
		for (int i0 = 0; i0 < Raw->DDObs.Sats; i0++)//��˫��۲�ֵ����ѭ��
		{
			if (Raw->DDObs.ddSatObs[i0].System == GPS) sys = 0;
			else if (Raw->DDObs.ddSatObs[i0].System == BDS) sys = 1;

			if (Raw->DDObs.ddSatObs[i0].Valid != true) continue;
			int di = Raw->DDObs.ddSatObs[i0].Pos;//��ǰ˫�������ڵ��������е�λ��
			int RovIndex = Raw->SdObs.SdSatObs[di].nRov;//��ǰ��������������վ�۲��е�λ��
			int RovRefIndex = Raw->DDObs.RefPos[sys];//�ο������ڵ����е����
			double dPos[3] = { 0 };//ÿ��˫��۲����ǵ�����վ�ľ���
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
		//��С���˽���
		MatrixTranspose((validsat * 2), 3, B, BT);
		MatrixMultiply_ATPPA((validsat * 2), 3,(validsat * 2), B, P, NBB);
		MatrixMultiply_APPB(3, (validsat * 2), (validsat * 2), 1, BT, P, w, W);
		lu_decomposition_inverse(3, NBB, Qxx);
		MatrixMultiply(3, 3, 3, 1, Qxx, W, x);
		//��λ��������
		MatrixMultiply((validsat * 2), 3, 3, 1, B, x, Bx);
		MatrixSubtraction((validsat * 2), 1, Bx, w, v);
		MatrixMultiply_ATPPA((validsat * 2), 1, (validsat * 2), v, P, &sigma);
		sigma0 = sqrt(sigma / (validsat * 2 - 3));
		pdop = sqrt(Qxx[0] + Qxx[4] + Qxx[8]);
		//��������վ��λ�ú�˫��ģ���Ȳ�����Ϊ��һ��ѭ����׼��
		dRcv[0] = x[0];
		dRcv[1] = x[1];
		dRcv[2] = x[2];
		MatrixAddition2(3, 1, dRcv, RovPos);
		Iterator++;
		//��̬�����ͷ�
		delete[]B;
		delete[]w;
		delete[]P;
		delete[]BT;
		delete[]Bx;
		delete[]v;
	} while (Norm(3, dRcv) > 1e-8 && Iterator < 10);
	//Raw->DDObs.Sats = validsat;//��Ч�۲�˫��������
	//Raw->DDObs.DDSatNum[0] = gpscount;//��Ч�۲�GPS˫��������
	//Raw->DDObs.DDSatNum[1] = bdscount;//��Ч�۲�BDS˫��������
	MatrixSubtraction(3, 1, RovPos, BasPos, Raw->DDObs.dPos);// ��������
	memcpy(Raw->DDObs.RovXYZ, RovPos, 3 * sizeof(double));
	XYZToBLH(RovPos, Raw->DDObs.RovBLH, R_WGS84, F_WGS84);
	Comp_dEnu(BasPos, RovPos, Raw->DDObs.dENH);
	Raw->DDObs.FixRMS[0] = sigma0 * pdop;
}