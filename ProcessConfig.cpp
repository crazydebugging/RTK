#include "RTK_Structs.h"
/****************************************************************************
  ReadSATODSConfigInfo

  Ŀ�ģ���ȡ�����ļ�

  ��ţ�001

  ����:
  cfg			������Ϣ�ṹ��


  ����ֵ�����óɹ�����1������ʧ�ܷ���0
****************************************************************************/
bool ReadSATODSConfigInfo(ROVERCFGINFO& cfg)
{
	cfg.IsFileData = 1;		// 1=FILE, 0=COM
	cfg.RTKProcMode = 2;	//1 = EKF, 2 = LSQ

	//strcpy_s(cfg.BasObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202202030930-1.bin");//  �۲����ݵ��ļ��������
	//strcpy_s(cfg.RovObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202202030930-2.bin");
	//strcpy_s(cfg.BasObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202203170900-1.bin");//  �۲����ݵ��ļ����̻���
	//strcpy_s(cfg.RovObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\oem719-202203170900-2.bin");
	strcpy_s(cfg.BasObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\rtkBASE.oem719");//  �۲����ݵ��ļ���8Сʱ
	strcpy_s(cfg.RovObsDatFile, "D:\\2022Project\\C\\RTK\\DATA\\rtkROVER.oem719");

	strcpy_s(cfg.ResFile, "D:\\2022Project\\C\\RTK\\DATA\\result\\pos.txt");//  ��������ļ���
	strcpy_s(cfg.ResFile1, "D:\\2022Project\\C\\RTK\\DATA\\result\\pos1.txt");//  ��������ļ���

	strcpy_s(cfg.BasNetIP, "47.114.134.129");//  ��վ����
	strcpy_s(cfg.RovNetIP, "8.140.46.126");//  ����վ����
	cfg.BasNetPort = 7190;//  ��վ�˿�
	cfg.RovNetPort = 4002;//  ����վ����

	return true;
}