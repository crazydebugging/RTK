// main.cpp : 此文件包含 "main" 函数。
#define _CRT_SECURE_NO_WARNINGS
#include "RTK_Structs.h"

int main()
{
	//结构体定义
	/*unsigned char Buff[MAXRAWLEN * 3];*/
	RAWDAT Raw;//RTK定位数据
	PPRESULT BResult, RResult;//基站和流动站SPP结果
	RTKEKF EKF;//EKF滤波数据
	ROVERCFGINFO cfg;//配置信息
	ReadSATODSConfigInfo(cfg);//读取配置信息
	FILE* FBas=NULL; FILE* FRov=NULL;//基站和移动站文件指针
	SOCKET BasSock, RovSock;//基站和移动站串口
	int obsnum = 0;
	//SPP结果文件
	FILE* file1 = fopen("D:\\2022Project\\C\\RTK\\DATA\\result\\Bpos.txt", "w");
	if (file1 == NULL) {
		printf("无法打开文件Bpos。\n");
		return 1;
	}
	FILE* file2 = fopen("D:\\2022Project\\C\\RTK\\DATA\\result\\Rpos.txt", "w");
	if (file2 == NULL) {
		printf("无法打开文件Rpos。\n");
		return 1;
	}
	//RTK结果文件
	FILE* file3 = fopen(cfg.ResFile, "w");
	if (file3 == NULL) {
		printf("无法打开文件。\n");
		return 1;
	}
	FILE* file4 = fopen(cfg.ResFile1, "w");
	if (file4 == NULL) {
		printf("无法打开文件。\n");
		return 1;
	}
	fprintf(file1, "  Wk        SOW       ECEF-X/m       ECEF-Y/m       ECEF-Z/m    REF-ECEF-X/m    REF-ECEF-Y/m   REF-ECEF-Z/m   EAST/m   NORTH/m  UP/m         B/deg         L/deg             H/m      VX/m     VY/m     VZ/m     PDOP    SigmaP   SigmaV  GS  BS  n\n");
	fprintf(file2, "  Wk        SOW       ECEF-X/m       ECEF-Y/m       ECEF-Z/m    REF-ECEF-X/m    REF-ECEF-Y/m   REF-ECEF-Z/m   EAST/m   NORTH/m  UP/m         B/deg         L/deg             H/m      VX/m     VY/m     VZ/m     PDOP    SigmaP   SigmaV  GS  BS  n\n");
	/*fprintf(file3, "  Wk        SOW         dX/m      dY/m      dZ/m         dE/m      dN/m      dH/m      RMS    Ratio\n");*/

	if (cfg.IsFileData == 1)
	{
		if ((FBas = fopen(cfg.BasObsDatFile, "rb")) == NULL)//基站文件指针
		{
			printf("Cannot open Base GPS obs file. \n");
			return 0;
		}
		if ((FRov = fopen(cfg.RovObsDatFile, "rb")) == NULL)//移动站文件指针
		{
			printf("Cannot open Rove GPS obs file. \n");
			return 0;
		}
		while (!(feof(FBas) || feof(FRov)))
		{
			if (GetSynObs(FBas, FRov, BasSock, RovSock,&Raw))//数据解码与时间同步,获取同步结果至Raw中
			{
				// 处理观测值数据,进行定位,定位失败暂不考虑
				if (SPP(&(Raw.RovEpk), &Raw, &RResult) && SPP(&(Raw.BasEpk), &Raw, &BResult))
				{
					//obsnum++; printf("%d\n", obsnum);
					//if (obsnum == 2994)//34465
					//{
					//	printf("定位失败\n");
					//}
					//单点定位的结果
					Raw.DDObs.Ratio = 0;//Ratio值
					MatrixSubtraction(3, 1, RResult.Position, Raw.BasEpk.Pos, Raw.DDObs.dPos);// 基线向量
					memcpy(Raw.DDObs.RovXYZ, RResult.Position, 3 * sizeof(double));
					/*XYZToBLH(Raw.BasEpk.Pos, Raw.DDObs.RovBLH, R_WGS84, F_WGS84);*/
					XYZToBLH(RResult.Position, Raw.DDObs.RovBLH, R_WGS84, F_WGS84);
					Comp_dEnu(Raw.BasEpk.Pos, RResult.Position, Raw.DDObs.dENH);
					Raw.DDObs.FixRMS[0] = RResult.SigmaPos;// 浮点解定位中rms误差

					if (cfg.RTKProcMode == 2)RTKFloat(&Raw, &BResult, &RResult);//双差RTKLSQ
					else if (cfg.RTKProcMode == 1)
					{
						RTKekf(&Raw, &RResult, &EKF);//双差RTKEKF
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
					/*printf("定位失败");*/
				}
			}
		}
		// 关闭文件
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
			if (GetSynObs(FBas, FRov, BasSock, RovSock,&Raw))//数据解码与时间同步,获取同步结果至Raw中
			{
				// 处理观测值数据,进行定位,定位失败暂不考虑
				if (SPP(&(Raw.RovEpk), &Raw, &RResult) && SPP(&(Raw.BasEpk), &Raw, &BResult))
				{
					/*obsnum++; printf("%d\n", obsnum);*/
					//if (obsnum == 34465)
					//{
					//	printf("定位失败\n");
					//}
					//单点定位的结果
					Raw.DDObs.Ratio = 0;//Ratio值
					MatrixSubtraction(3, 1, RResult.Position, Raw.BasEpk.Pos, Raw.DDObs.dPos);// 基线向量
					Comp_dEnu(Raw.BasEpk.Pos, RResult.Position, Raw.DDObs.dENH);
					Raw.DDObs.FixRMS[0] = RResult.SigmaPos;// 浮点解定位中rms误差

					if (cfg.RTKProcMode == 2)RTKFloat(&Raw, &BResult, &RResult);//双差RTKLSQ
					else if (cfg.RTKProcMode == 1)
					{
						RTKekf(&Raw, &RResult, &EKF);//双差RTKEKF
					}
					/*fprintf(file3, "%hu %10.3f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f   %5.3f   %5.3f "
						, RResult.Time.Week, RResult.Time.SecOfWeek, Raw.DDObs.dPos[0], Raw.DDObs.dPos[1], Raw.DDObs.dPos[2], Raw.DDObs.dENH[0], Raw.DDObs.dENH[1], Raw.DDObs.dENH[2], Raw.DDObs.FixRMS[0], Raw.DDObs.Ratio);*/
					printf("%hu %10.3f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f   %5.3f   %5.3f "
						, RResult.Time.Week, RResult.Time.SecOfWeek, Raw.DDObs.dPos[0], Raw.DDObs.dPos[1], Raw.DDObs.dPos[2], Raw.DDObs.dENH[0], Raw.DDObs.dENH[1], Raw.DDObs.dENH[2], Raw.DDObs.FixRMS[0], Raw.DDObs.Ratio);
					if (Raw.DDObs.IsSuccess == false)
					{
						/*fprintf(file3, "\t单点定位");*/ printf("\t单点定位"); fprintf(file3, "%d",5);
					}
					else if (Raw.DDObs.bFixed == false)
					{
						/*fprintf(file3, "\t浮点解");*/ printf( "\t浮点解"); fprintf(file3, "%d", 2);
					}
					else
					{
						/*fprintf(file3, "\t固定解");*/ printf("\t固定解"); fprintf(file3, "%d", 1);
					}
					fprintf(file3, "\n"); printf( "\n");
				}
				else
				{
					/*printf("定位失败");*/
				}
			}
		}
	}
	return 1;
}
/*obsnum++; printf("%d\n", obsnum);*/
/*if (obsnum == 1036)
{
   printf("定位失败\n");
}*/
/*fprintf(file1, "%hu %10.3f %14.4f %14.4f %14.4f %14.4f %14.4f %14.4f %6.3f %6.3f %6.3f %11.8f %12.8f %6.3f %6.3f %6.3f %6.3f %5.3f %5.3f %5.3f %d %d %d "
					, BResult.Time.Week, BResult.Time.SecOfWeek, BResult.Position[0], BResult.Position[1], BResult.Position[2], Raw.BasEpk.Pos[0], Raw.BasEpk.Pos[1], Raw.BasEpk.Pos[2], BResult.dENH[0], BResult.dENH[1], BResult.dENH[2], BResult.BLH[0] * Deg, BResult.BLH[1] * Deg, BResult.BLH[2],
					BResult.Velocity[0], BResult.Velocity[1], BResult.Velocity[2], BResult.PDOP, BResult.SigmaPos, BResult.SigmaVel, BResult.GPSSatNum, BResult.BDSSatNum, BResult.AllSatNum);
				fprintf(file1, "\n");
fprintf(file2, "%hu %10.3f %14.4f %14.4f %14.4f %14.4f %14.4f %14.4f %6.3f %6.3f %6.3f %11.8f %12.8f %6.3f %6.3f %6.3f %6.3f %5.3f %5.3f %5.3f %d %d %d "
					, RResult.Time.Week, RResult.Time.SecOfWeek, RResult.Position[0], RResult.Position[1], RResult.Position[2], Raw.RovEpk.Pos[0], Raw.RovEpk.Pos[1], Raw.RovEpk.Pos[2], RResult.dENH[0], RResult.dENH[1], RResult.dENH[2], RResult.BLH[0] * Deg, RResult.BLH[1] * Deg, RResult.BLH[2],
					RResult.Velocity[0], RResult.Velocity[1], RResult.Velocity[2], RResult.PDOP, RResult.SigmaPos, RResult.SigmaVel, RResult.GPSSatNum, RResult.BDSSatNum, RResult.AllSatNum);
				fprintf(file2, "\n");*/