#include "RTK_Structs.h"
/****************************************************************************
  ReadSATODSConfigInfo

  目的：读取配置文件

  编号：001

  参数:
  cfg			配置信息结构体


  返回值：配置成功返回1，配置失败返回0
****************************************************************************/
bool ReadSATODSConfigInfo(ROVERCFGINFO& cfg)
{
	cfg.IsFileData = 1;		// 1=FILE, 0=COM
	cfg.RTKProcMode = 2;	//1 = EKF, 2 = LSQ

	//strcpy_s(cfg.BasObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202202030930-1.bin");//  观测数据的文件名零基线
	//strcpy_s(cfg.RovObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202202030930-2.bin");
	//strcpy_s(cfg.BasObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202203170900-1.bin");//  观测数据的文件名短基线
	//strcpy_s(cfg.RovObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202203170900-2.bin");
	strcpy_s(cfg.BasObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\rtkBASE.oem719");//  观测数据的文件名8小时
	strcpy_s(cfg.RovObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\rtkROVER.oem719");

	strcpy_s(cfg.ResFile, "D:\\2022Project\\C\\RTK\\DATA\\result\\pos.txt");//  结果数据文件名
	strcpy_s(cfg.ResFile1, "D:\\2022Project\\C\\RTK\\DATA\\result\\pos1.txt");//  结果数据文件名

	strcpy_s(cfg.BasNetIP, "47.114.134.129");//  基站网口
	strcpy_s(cfg.RovNetIP, "8.140.46.126");//  流动站网口
	cfg.BasNetPort = 7190;//  基站端口
	cfg.RovNetPort = 4002;//  流动站网口

	return true;
}