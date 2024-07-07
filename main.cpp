// main.cpp : ���ļ����� "main" ������
#define _CRT_SECURE_NO_WARNINGS
#include "RTK_Structs.h"

int main()
{
	//�ṹ�嶨��
	/*unsigned char Buff[MAXRAWLEN * 3];*/
	RAWDAT Raw;//RTK��λ����
	PPRESULT BResult, RResult;//��վ������վSPP���
	RTKEKF EKF;//EKF�˲�����
	ROVERCFGINFO cfg;//������Ϣ
	ReadSATODSConfigInfo(cfg);//��ȡ������Ϣ
	FILE* FBas=NULL; FILE* FRov=NULL;//��վ���ƶ�վ�ļ�ָ��
	SOCKET BasSock, RovSock;//��վ���ƶ�վ����
	int obsnum = 0;
	//SPP����ļ�
	FILE* file1 = fopen("D:\\2022Project\\C\\RTK\\DATA\\result\\Bpos.txt", "w");
	if (file1 == NULL) {
		printf("�޷����ļ�Bpos��\n");
		return 1;
	}
	FILE* file2 = fopen("D:\\2022Project\\C\\RTK\\DATA\\result\\Rpos.txt", "w");
	if (file2 == NULL) {
		printf("�޷����ļ�Rpos��\n");
		return 1;
	}
	//RTK����ļ�
	FILE* file3 = fopen(cfg.ResFile, "w");
	if (file3 == NULL) {
		printf("�޷����ļ���\n");
		return 1;
	}
	FILE* file4 = fopen(cfg.ResFile1, "w");
	if (file4 == NULL) {
		printf("�޷����ļ���\n");
		return 1;
	}
	fprintf(file1, "  Wk        SOW       ECEF-X/m       ECEF-Y/m       ECEF-Z/m    REF-ECEF-X/m    REF-ECEF-Y/m   REF-ECEF-Z/m   EAST/m   NORTH/m  UP/m         B/deg         L/deg             H/m      VX/m     VY/m     VZ/m     PDOP    SigmaP   SigmaV  GS  BS  n\n");
	fprintf(file2, "  Wk        SOW       ECEF-X/m       ECEF-Y/m       ECEF-Z/m    REF-ECEF-X/m    REF-ECEF-Y/m   REF-ECEF-Z/m   EAST/m   NORTH/m  UP/m         B/deg         L/deg             H/m      VX/m     VY/m     VZ/m     PDOP    SigmaP   SigmaV  GS  BS  n\n");
	/*fprintf(file3, "  Wk        SOW         dX/m      dY/m      dZ/m         dE/m      dN/m      dH/m      RMS    Ratio\n");*/

	if (cfg.IsFileData == 1)
	{
		if ((FBas = fopen(cfg.BasObsDatFile, "rb")) == NULL)//��վ�ļ�ָ��
		{
			printf("Cannot open Base GPS obs file. \n");
			return 0;
		}
		if ((FRov = fopen(cfg.RovObsDatFile, "rb")) == NULL)//�ƶ�վ�ļ�ָ��
		{
			printf("Cannot open Rove GPS obs file. \n");
			return 0;
		}
		while (!(feof(FBas) || feof(FRov)))
		{
			if (GetSynObs(FBas, FRov, BasSock, RovSock,&Raw))//���ݽ�����ʱ��ͬ��,��ȡͬ�������Raw��
			{
				// ����۲�ֵ����,���ж�λ,��λʧ���ݲ�����
				if (SPP(&(Raw.RovEpk), &Raw, &RResult) && SPP(&(Raw.BasEpk), &Raw, &BResult))
				{
					//obsnum++; printf("%d\n", obsnum);
					//if (obsnum == 2994)//34465
					//{
					//	printf("��λʧ��\n");
					//}
					//���㶨λ�Ľ��
					Raw.DDObs.Ratio = 0;//Ratioֵ
					MatrixSubtraction(3, 1, RResult.Position, Raw.BasEpk.Pos, Raw.DDObs.dPos);// ��������
					memcpy(Raw.DDObs.RovXYZ, RResult.Position, 3 * sizeof(double));
					/*XYZToBLH(Raw.BasEpk.Pos, Raw.DDObs.RovBLH, R_WGS84, F_WGS84);*/
					XYZToBLH(RResult.Position, Raw.DDObs.RovBLH, R_WGS84, F_WGS84);
					Comp_dEnu(Raw.BasEpk.Pos, RResult.Position, Raw.DDObs.dENH);
					Raw.DDObs.FixRMS[0] = RResult.SigmaPos;// ����ⶨλ��rms���

					if (cfg.RTKProcMode == 2)RTKFloat(&Raw, &BResult, &RResult);//˫��RTKLSQ
					else if (cfg.RTKProcMode == 1)
					{
						RTKekf(&Raw, &RResult, &EKF);//˫��RTKEKF
					}
					if (Raw.DDObs.IsSuccess == false)
					{
						Raw.DDObs.flag = 5;
						continue;
					}
					else if (Raw.DDObs.IsSuccess == true&&Raw.DDObs.bFixed == false)
					{
						Raw.DDObs.flag = 2;
					}
					else if(Raw.DDObs.IsSuccess == true && Raw.DDObs.bFixed == true)
					{
						Raw.DDObs.flag = 1;
					}
					/*fprintf(file3, "%hu %10.3f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f   %5.3f   %5.3f %d"
						, RResult.Time.Week, RResult.Time.SecOfWeek, Raw.DDObs.dPos[0], Raw.DDObs.dPos[1], Raw.DDObs.dPos[2], Raw.DDObs.dENH[0], Raw.DDObs.dENH[1], Raw.DDObs.dENH[2], Raw.DDObs.FixRMS[0], Raw.DDObs.Ratio,Raw.DDObs.flag);*/
					fprintf(file3, "%hu %10.3f %12.9f %13.9f %8.4f %d %d %d %d %d %d %d %d %5.3f %5.3f"
						, RResult.Time.Week, RResult.Time.SecOfWeek, Raw.DDObs.RovBLH[0]*Deg, Raw.DDObs.RovBLH[1] * Deg, Raw.DDObs.RovBLH[2], Raw.DDObs.flag, Raw.DDObs.Sats, 0, 0, 0, 0, 0, 0, Raw.DDObs.FixRMS[0], Raw.DDObs.Ratio);
					fprintf(file4, "%hu %10.3f %12.9f %13.9f %8.4f %d %d %5.3f %d %d %d %d %d %d %5.3f"
						, RResult.Time.Week, RResult.Time.SecOfWeek, Raw.DDObs.RovXYZ[0], Raw.DDObs.RovXYZ[1], Raw.DDObs.RovXYZ[2], Raw.DDObs.flag,Raw.DDObs.Sats, Raw.DDObs.FixRMS[0],0,0,0,0,0,0, Raw.DDObs.Ratio);
					fprintf(file3, "\n");
					fprintf(file4, "\n");
				}
				else
				{
					/*printf("��λʧ��");*/
				}
			}
		}
		// �ر��ļ�
		fclose(FBas);
		fclose(FRov);
	}
	else if (cfg.IsFileData == 0)
	{
		if (OpenSocket(BasSock, cfg.BasNetIP, cfg.BasNetPort) == false) {
			printf("This ip & port was not opened.\n");
			return 0;
		}
		if (OpenSocket(RovSock, cfg.RovNetIP, cfg.RovNetPort) == false) {
			printf("This ip & port was not opened.\n");
			return 0;
		}
		while (1)
		{
			if (GetSynObs(FBas, FRov, BasSock, RovSock,&Raw))//���ݽ�����ʱ��ͬ��,��ȡͬ�������Raw��
			{
				// ����۲�ֵ����,���ж�λ,��λʧ���ݲ�����
				if (SPP(&(Raw.RovEpk), &Raw, &RResult) && SPP(&(Raw.BasEpk), &Raw, &BResult))
				{
					/*obsnum++; printf("%d\n", obsnum);*/
					//if (obsnum == 34465)
					//{
					//	printf("��λʧ��\n");
					//}
					//���㶨λ�Ľ��
					Raw.DDObs.Ratio = 0;//Ratioֵ
					MatrixSubtraction(3, 1, RResult.Position, Raw.BasEpk.Pos, Raw.DDObs.dPos);// ��������
					Comp_dEnu(Raw.BasEpk.Pos, RResult.Position, Raw.DDObs.dENH);
					Raw.DDObs.FixRMS[0] = RResult.SigmaPos;// ����ⶨλ��rms���

					if (cfg.RTKProcMode == 2)RTKFloat(&Raw, &BResult, &RResult);//˫��RTKLSQ
					else if (cfg.RTKProcMode == 1)
					{
						RTKekf(&Raw, &RResult, &EKF);//˫��RTKEKF
					}
					/*fprintf(file3, "%hu %10.3f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f   %5.3f   %5.3f "
						, RResult.Time.Week, RResult.Time.SecOfWeek, Raw.DDObs.dPos[0], Raw.DDObs.dPos[1], Raw.DDObs.dPos[2], Raw.DDObs.dENH[0], Raw.DDObs.dENH[1], Raw.DDObs.dENH[2], Raw.DDObs.FixRMS[0], Raw.DDObs.Ratio);*/
					printf("%hu %10.3f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f   %5.3f   %5.3f "
						, RResult.Time.Week, RResult.Time.SecOfWeek, Raw.DDObs.dPos[0], Raw.DDObs.dPos[1], Raw.DDObs.dPos[2], Raw.DDObs.dENH[0], Raw.DDObs.dENH[1], Raw.DDObs.dENH[2], Raw.DDObs.FixRMS[0], Raw.DDObs.Ratio);
					if (Raw.DDObs.IsSuccess == false)
					{
						/*fprintf(file3, "\t���㶨λ");*/ printf("\t���㶨λ"); fprintf(file3, "%d",5);
					}
					else if (Raw.DDObs.bFixed == false)
					{
						/*fprintf(file3, "\t�����");*/ printf( "\t�����"); fprintf(file3, "%d", 2);
					}
					else
					{
						/*fprintf(file3, "\t�̶���");*/ printf("\t�̶���"); fprintf(file3, "%d", 1);
					}
					fprintf(file3, "\n"); printf( "\n");
				}
				else
				{
					/*printf("��λʧ��");*/
				}
			}
		}
	}
	return 1;
}
/*obsnum++; printf("%d\n", obsnum);*/
/*if (obsnum == 1036)
{
   printf("��λʧ��\n");
}*/
/*fprintf(file1, "%hu %10.3f %14.4f %14.4f %14.4f %14.4f %14.4f %14.4f %6.3f %6.3f %6.3f %11.8f %12.8f %6.3f %6.3f %6.3f %6.3f %5.3f %5.3f %5.3f %d %d %d "
					, BResult.Time.Week, BResult.Time.SecOfWeek, BResult.Position[0], BResult.Position[1], BResult.Position[2], Raw.BasEpk.Pos[0], Raw.BasEpk.Pos[1], Raw.BasEpk.Pos[2], BResult.dENH[0], BResult.dENH[1], BResult.dENH[2], BResult.BLH[0] * Deg, BResult.BLH[1] * Deg, BResult.BLH[2],
					BResult.Velocity[0], BResult.Velocity[1], BResult.Velocity[2], BResult.PDOP, BResult.SigmaPos, BResult.SigmaVel, BResult.GPSSatNum, BResult.BDSSatNum, BResult.AllSatNum);
				fprintf(file1, "\n");
fprintf(file2, "%hu %10.3f %14.4f %14.4f %14.4f %14.4f %14.4f %14.4f %6.3f %6.3f %6.3f %11.8f %12.8f %6.3f %6.3f %6.3f %6.3f %5.3f %5.3f %5.3f %d %d %d "
					, RResult.Time.Week, RResult.Time.SecOfWeek, RResult.Position[0], RResult.Position[1], RResult.Position[2], Raw.RovEpk.Pos[0], Raw.RovEpk.Pos[1], Raw.RovEpk.Pos[2], RResult.dENH[0], RResult.dENH[1], RResult.dENH[2], RResult.BLH[0] * Deg, RResult.BLH[1] * Deg, RResult.BLH[2],
					RResult.Velocity[0], RResult.Velocity[1], RResult.Velocity[2], RResult.PDOP, RResult.SigmaPos, RResult.SigmaVel, RResult.GPSSatNum, RResult.BDSSatNum, RResult.AllSatNum);
				fprintf(file2, "\n");*/