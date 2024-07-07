// time.cpp : ���ļ�����ʱ��ת��������
//
#include "RTK_Structs.h"

//ͨ��ʱת��Ϊ��������
void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT)
{
	int y, m;
	if (CT->Month <= 2)
	{
		y = CT->Year - 1;
		m = CT->Month + 12;
	}
	else
	{
		y = CT->Year;
		m = CT->Month;
	}
	//ǰ���������
	MJDT->Days = (int)((int)(365.25*y) + (int)(30.6001*((double)m + 1)) + CT->Day + 1720981.5 - 2400000.5);
	//��������ʱ����Ҳ��Ϊ��
	MJDT->FracDay= 1.0*CT->Hour / 24 + 1.0*CT->Minute / 24 / 60 + CT->Second / 24 / 3600;
}

//��������ת��Ϊͨ��ʱ
void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT)
{
	double JD;
	JD = MJDT->Days + MJDT->FracDay + 2400000.5;
	
	int a, b, c, d, e,n;
	a = int(JD + 0.5);
	b = a + 1537;
	c = int((b - 122.1) / 365.25);
	d = int(365.25 * c);
	e = int((b - d) / 30.6001);
	n = int(MJDT->FracDay * 3600*24);

	CT->Day = b - d - int(30.6001 * e);
	CT->Month = e - 1 - 12 * int(e / 14);
	CT->Year = c - 4715 - int((7 + CT->Month) / 10);

	CT->Hour = n / 3600;
	CT->Minute = n % 3600 / 60;
	CT->Second = n % 60;

}

//��������ת��ΪGPSʱ
void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT)
{
	GT->Week = int((MJDT->Days+MJDT->FracDay - 44244) / 7);
	GT->SecOfWeek = (MJDT->Days + MJDT->FracDay - 44244 - GT->Week * 7) * 86400;
}

//GPSʱת��Ϊ��������
void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT)
{
	MJDT->Days = 44244 + GT->Week * 7 + int(GT->SecOfWeek / 86400);
	MJDT->FracDay = (GT->SecOfWeek - int(GT->SecOfWeek / 86400) * 86400)/86400;
}

//ͨ��ʱת��ΪGPSʱ
void CommonTimeToGPSTime(const COMMONTIME* CT, GPSTIME* GT)
{
	MJDTIME mjdt;
	CommonTimeToMJDTime(CT, &mjdt);
	MJDTimeToGPSTime(&mjdt, GT);
}

//GPSʱת��Ϊͨ��ʱ
void GPSTimeToCommonTime(const GPSTIME* GT, COMMONTIME* CT)
{
	MJDTIME mjdt;
	GPSTimeToMJDTime(GT, &mjdt);
	MJDTimeToCommonTime(&mjdt, CT);
}

//������GPSʱ��ʱ��
double GetDiffTime(const GPSTIME* GT2, const GPSTIME* GT1)
{
	double temp1,temp2;
	temp1 = (GT2->Week - GT1->Week) * 604800;
	temp2 = GT2->SecOfWeek - GT1->SecOfWeek;
	return temp1 + temp2;
}