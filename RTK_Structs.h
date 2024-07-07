/****************************************************************************
Ŀ�ģ�    ����GPS+BDS RTK�����Ҫ�ĳ����ͽṹ��
��дʱ�䣺2022.1.10
���ߣ�    ������
�汾:     V1.0
��Ȩ��    �人��ѧ���ѧԺ
****************************************************************************/

#include<fstream>
#include<windows.h>
#include <omp.h>  // ����OpenMP��
#ifndef _GNSS_RTK_H_
#define _GNSS_RTK_H_

#define PAI 3.1415926535898
#define PAI2 (2.0*PAI)                    /* 2pi */
#define Rad (PAI/180.0)                  /* Radians per degree */
#define Deg (180.0/PAI)                  /* Degrees per radian */
#define C_Light 299792458.0      /* Speed of light  [m/s]; IAU 1976  */

#define R_WGS84  6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84   */
#define Omega_WGS 7.2921151467e-5   /*[rad/s], the earth rotation rate */
#define GM_Earth   398600.5e+9      /* [m^3/s^2]; WGS-84 */
#define R_CGS2K  6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K  1.0/298.257222101  /* Flattening; CGCS2000   */
#define Omega_BDS 7.2921150e-5      /*[rad/s], the earth rotation rate */
#define GM_BDS   398600.4418e+9     /* [m^3/s^2]; CGCS2000  */

/* some constants about GPS satellite signal */
#define  FG1_GPS  1575.42E6             /* L1�ź�Ƶ�� */
#define  FG2_GPS  1227.60E6             /* L2�ź�Ƶ�� */
#define  FG12R    (77/60.0)             /* FG1_Freq/FG2_Freq */
#define  FG12R2   (5929/3600.0)
#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)

/* some constants about Compass satellite signal */
#define  FG1_BDS  1561.098E6               /* B1�źŵĻ�׼Ƶ�� */
#define  FG2_BDS  1207.140E6               /* B2�źŵĻ�׼Ƶ�� */
#define  FG3_BDS  1268.520E6               /* B3�źŵĻ�׼Ƶ�� */
#define  FC12R    (FG1_BDS/FG2_BDS)       /* FG1_BDS/FG2_BDS */
#define  FC12R2   (FC12R*FC12R)           /* FG1_BDS^2/FG2_BDS^2 */
#define  FC13R    (FG1_BDS/FG3_BDS)       /* FG1_BDS^2/FG3_BDS^2 */
#define  FC13R2   (FC13R*FC13R)
#define  WL1_BDS  (C_Light/FG1_BDS)
#define  WL2_BDS  (C_Light/FG2_BDS)
#define  WL3_BDS  (C_Light/FG3_BDS)

#define GPST_BDT  14         /* GPSʱ�뱱��ʱ�Ĳ�ֵ[s] */
#define MAXCHANNUM 36
#define MAXGPSNUM  32
#define MAXBDSNUM 63
#define MAXRAWLEN 40960


/* ��������ϵͳ���� */
enum GNSSSys { UNKS=0, GPS, BDS};

struct COMMONTIME   /* ͨ��ʱ�䶨�� */
{
    short Year;
    unsigned short Month;
    unsigned short Day;
    unsigned short Hour;
    unsigned short Minute;
    double         Second;
    
	COMMONTIME()
	{
		Year	= 0;
		Month	= 0;
		Day		= 0;
		Hour	= 0;
		Minute	= 0;
		Second	= 0.0;
	}
};

struct GPSTIME              /* GPSʱ�䶨�� */
{
    unsigned short Week;          
    double         SecOfWeek;
    
    GPSTIME()
    {
        Week = 0;
        SecOfWeek = 0.0;
    }
};

struct MJDTIME             /* �������� */
{
    int Days;             
    double FracDay;
    
    MJDTIME()
    {
        Days = 0;
        FracDay = 0.0;
    }
};

// GPS+BDS�㲥����
struct GPSEPHREC
{
	unsigned short PRN;
	GNSSSys     Sys;
	GPSTIME  	TOC, TOE;
	short		SVHealth;
	double		ClkBias, ClkDrift, ClkDriftRate;
	unsigned long		IODE, IODC;
	double      TGD1, TGD2;
	double		SqrtA, e, M0, OMEGA, i0, omega, OMEGADot, iDot, DeltaN;
	double		Crs, Cuc, Cus, Cic, Cis, Crc;
    double		SVAccuracy;

	GPSEPHREC() {
		PRN = SVHealth = 0;
		Sys = UNKS;
		ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
		SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
		Crs = Cuc = Cus = Cic = Cis = Crc = SVAccuracy = 0.0;
	}
};

/*  ÿ�����ǵĹ۲����ݶ���  */
struct SATOBS
{
    short    Prn;
  	GNSSSys  System;
	double   P[2], L[2], D[2];   // m
	double   cn0[2], LockTime[2];
	bool half[2];
	/*unsigned char half[2];*/
	bool     Valid;
   
    SATOBS()
    {
        Prn = 0;
        System = UNKS;
		for (int i = 0; i < 2; i++)
		{
			P[i] = L[i] = D[i] = cn0[i]= LockTime[i]=0;
			half[i] = false;
		}
		Valid = false;
    }
};
struct MWGF
{
	short Prn;//���Ǻ�
	GNSSSys Sys;
	double MW, GF, PIF;

	int n;//ƽ������

	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};

/* ÿ������λ�á��ٶȺ��Ӳ�ȵ��м������ */
struct SATMIDRES
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid;  //false=û����������������,true-����ɹ�

	SATMIDRES()
	{
		SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
		SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
		Elevation = PAI / 2.0;
		Azimuth = 0.0;
		SatClkOft = SatClkSft = 0.0;
		Tgd1 = Tgd2 = TropCorr = 0.0;
		Valid = false;
	}
};
/*  ÿ����Ԫ�Ĺ۲����ݶ���  */
struct EPOCHOBS
{
	GPSTIME    Time;
	short      SatNum;
	SATOBS     SatObs[MAXCHANNUM];
	SATMIDRES  SatPVT[MAXCHANNUM]; // ����λ�õȼ�����������������SatObs��ͬ
	MWGF       ComObs[MAXCHANNUM];  // ��ǰ��Ԫ����Ϲ۲�ֵ������������SatObs��ͬ
	double     Pos[3];      // �����վ��NovAtel���ջ���λ���		deg,deg,m������תΪXYZ

	EPOCHOBS()
	{
		SatNum = 0;
		Pos[0] = Pos[1] = Pos[2] = 0.0;
	}
};

/*  ÿ�����ǵĵ���۲����ݶ���  */
struct SDSATOBS
{
	short    Prn;
	GNSSSys  System;
	short    Valid;
	double   dP[2], dL[2];   // m
	short    nBas, nRov;   // �洢����۲�ֵ��Ӧ�Ļ�׼������վ����ֵ������

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKS;
		dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
		Valid = -1;
	}
};

/*  ÿ����Ԫ�ĵ���۲����ݶ���  */
struct SDEPOCHOBS
{
	GPSTIME    Time;
	short      SatNum;
	SDSATOBS   SdSatObs[MAXCHANNUM];
	MWGF       SdCObs[MAXCHANNUM];

	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};

/*  ÿ�����ǵ�˫��۲����ݶ���  */
struct DDSATOBS
{
	short    Prn,Pos;// �洢˫��۲�ֵ��Ӧ�ĵ������ǵ����Ǻźʹ洢λ��
	GNSSSys  System;
	short    Valid;
	double   ddP[2], ddL[2];   // m

	DDSATOBS()
	{
		Prn =Pos=0;
		System = UNKS;
		ddP[0] = ddL[0] = ddP[1] = ddL[1] = 0.0;
		Valid = -1;
	}
};
/*  ÿ����Ԫ��˫����ص����ݶ���  */
struct DDCOBS
{
	GPSTIME    Time;
	int RefPrn[2], RefPos[2];         // �ο������Ǻ���洢λ�ã�0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // ������˫��ģ����������0=GPS; 1=BDS
	DDSATOBS   ddSatObs[MAXCHANNUM];	
	MWGF       ddCObs[MAXCHANNUM];

	double FixedAmb[MAXCHANNUM * 4];  // ����˫Ƶ���Ž�[0,AmbNum]�ʹ��Ž�[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA������е�ģ���Ȳв�
	float  FixRMS[2];                 // �̶��ⶨλ��rms���
	double dPos[3];                   // ��������
	double dENH[3];					//��λ�������
	bool bFixed;                      //trueΪ�̶���falseΪδ�̶�
	bool IsSuccess;					/* RTK�Ƿ�ɹ�, 1Ϊ�ɹ��������и����, 0Ϊʧ�� */
	double RovBLH[3], RovXYZ[3];	//����վ����λ��BLH,XYZ
	int flag;							//�������
	

	DDCOBS()
	{
		int i;
		for (i = 0; i<2; i++) {
			DDSatNum[i] = 0;    // ������ϵͳ��˫������
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // ˫����������
		flag = 0;
		for (int i = 0; i < 3; i++)		dPos[i] = dENH[i] = RovBLH[i]= RovXYZ[i] = 0.0;
		ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
		bFixed = false;
		IsSuccess = false;
		for (i = 0; i<MAXCHANNUM * 2; i++)
		{
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
		}
	}
};

/* ÿ����Ԫ���㶨λ�Ͳ��ٵĽ�����侫��ָ�� */
struct PPRESULT
{
    GPSTIME Time;
    double Position[3];
    double Velocity[3];
    double RcvClkOft[2];               /* 0 ΪGPS�Ӳ�; 1=BDS�Ӳ� */
    double RcvClkSft;
    double PDOP, SigmaPos, SigmaVel;  // ����ָ��
	short  GPSSatNum, BDSSatNum;      /* ���㶨λʹ�õ�GPS������ */
	short  AllSatNum;                /* �۲���Ԫ������������   */
	bool   IsSuccess;                /* ���㶨λ�Ƿ�ɹ�, 1Ϊ�ɹ�, 0Ϊʧ�� */
	double BLH[3], dENH[3];			//���㶨λ�������

	PPRESULT()
	{
		for (int i=0; i<3; i++)		Position[i] = Velocity[i] = BLH[i]= dENH[i]= 0.0;
		RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
		PDOP = SigmaPos = SigmaVel = 999.9;
		GPSSatNum = BDSSatNum = AllSatNum = 0;
		IsSuccess = false;
	}
};
/* ÿ����Ԫ��ʱ���˲�״̬ */
struct RTKEKF
{
	GPSTIME Time;
	double X[3 + MAXCHANNUM * 2], P[(3 + MAXCHANNUM * 2)*(3 + MAXCHANNUM * 2)];
	int nSats, nPos[MAXCHANNUM];
	int Index[MAXCHANNUM], FixAmb[MAXCHANNUM];          // ��ǰ��Ԫʱ����º��ϸ���Ԫ�Ѿ��̶������ݵ�ģ���ȣ� 1=�ѹ̶���-1=δ�̶�����������
	DDCOBS DDObs, CurDDObs;           // ��һ����Ԫ�͵�ǰ��Ԫ��˫��۲�ֵ��Ϣ
	SDEPOCHOBS SDObs,CurSDObs;                 // ��һ����Ԫ�͵�ǰ��Ԫ�ĵ���۲�ֵ
	double X0[3 + MAXCHANNUM * 2], P0[(3 + MAXCHANNUM * 2)*(3 + MAXCHANNUM * 2)];  // ״̬����
	bool IsInit;                      // �˲��Ƿ��ʼ��

	RTKEKF() {
		IsInit = false;
		nSats = 0;
		for (int i = 0; i < MAXCHANNUM; i++) nPos[i] = Index[i] = FixAmb[i] = -1;
		for (int i = 0; i < 3 + MAXCHANNUM * 2; i++) {
			X[i] = X0[i] = 0.0;
			for (int j = 0; j < 3 + MAXCHANNUM * 2; j++) P[i*(3 + MAXCHANNUM * 2) + j] = P0[i*(3 + MAXCHANNUM * 2) + j] = 0.0;
		}
	}
};
/*  RTK��λ�����ݶ���  */
struct RAWDAT {
	EPOCHOBS BasEpk;
	EPOCHOBS RovEpk;
	SDEPOCHOBS SdObs;
	DDCOBS DDObs;
	GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];
	EPOCHOBS BBasEpk;
	EPOCHOBS BRovEpk;
	DDCOBS BDDObs;
};
struct ROVERCFGINFO   // ������Ϣ
{
	short  IsFileData, RTKProcMode;      // 1=FILE, 0=COM, 1=EKF, 2=LSQ
	int    RovPort, RovBaud;             // COM�˿�����
	char   BasNetIP[20], RovNetIP[20];   // ip address
	short  BasNetPort, RovNetPort;       // port
	double CodeNoise, CPNoise;           // α������
    double ElevThreshold;                // �߶Ƚ���ֵ
	double RatioThres;                   // Ratio������ֵ
    
	char  BasObsDatFile[256], RovObsDatFile[256];    //  �۲����ݵ��ļ���
	char  ResFile[256],ResFile1[256];            //  ��������ļ���

    
    ROVERCFGINFO()
    {
		IsFileData = RTKProcMode = 1;
		RovPort = RovBaud = BasNetPort = RovNetPort = 0;
		CodeNoise = CPNoise = ElevThreshold = 0.0;
		RatioThres = 3.0;
	}
};

bool ReadSATODSConfigInfo( ROVERCFGINFO& cfg);

/* �������㺯��*/
void outputMatrixToFile(double* matrix, int rows, int cols, const char* filename);
void copyArray(MWGF* source, MWGF* destination, int size);
void compressMatrix(int rowsToDelete, int colsToDelete, const double* originalMatrix, int originalRows, int originalCols, double* compressedMatrix);
bool MatrixMultiply( int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[] );
bool MatrixMultiply_APB( int m1, int n1, int m2, int n2, const double M1[], const double P[], const double M2[], double M3[] );
bool MatrixMultiply_APPB(int m1, int n1, int m2, int n2, const double* A, const double* P, const double* B, double* C);
bool MatrixMultiply_ATPPA(int m1, int n1, int n, const double A[], const double P[], double M2[]);
bool MatrixMultiply_ABAT(int m1, int n1, int n, const double A[], const double B[], double M2[]);
bool MatrixMultiply_ABAT_C(int m1, int n1, int n, int m, const double A[], const double B[], const double C[], double M2[]);
bool MatrixMultiply_ABC(int m1, int n1, int m2, int n2, int m3, int n3, const double A[], const double B[], const double C[], double M[]);
void MatrixAddition( int m, int n, const double M1[], const double M2[], double M3[] );
void MatrixAddition2( int m, int n, const double M1[], double M2[] );
void MatrixSubtraction( int m, int n, const double M1[], const double M2[], double M3[] );
void MatrixTranspose( int m, int n, const double M1[], double MT[] );
int MatrixInv(int n, double a[], double b[]);
int lu_decomposition_inverse(int n, double* a, double* b);
void AdjRow(int m, int n, int isAdd, int row, double val, double M[]);

/* �������㺯��*/
void VectorAddition(int n, const double v1[], const double v2[], double v3[]);
void VectorSubtraction(int n, const double v1[], const double v2[], double v3[]);
double VectDot( int m, int n, const double A[], const double B[]);
double Norm( const int n, const double A[]);
void CrossDot( int m, int n, const double A[], const double B[], double C[]);

/* ͨ��ʱ,GPSʱ�ͼ�������֮����໥ת������*/
void CommonTimeToMJDTime( const COMMONTIME* CT, MJDTIME* MJDT);
void MJDTimeToCommonTime( const MJDTIME* MJDT, COMMONTIME* CT );
void GPSTimeToMJDTime( const GPSTIME* GT, MJDTIME* MJDT );
void MJDTimeToGPSTime ( const MJDTIME* MJDT, GPSTIME* GT );
void CommonTimeToGPSTime ( const COMMONTIME* CT, GPSTIME* GT );
void GPSTimeToCommonTime ( const GPSTIME* GT, COMMONTIME* CT );
double GetDiffTime( const GPSTIME* GT2, const GPSTIME* GT1 );

/* �ռ�ֱ������,���������໥ת������ */
void XYZToBLH( const double xyz[3], double blh[3], const double R, const double F );
void BLHToXYZ( const double BLH[3], double XYZ[3], const double R, const double F );
void BLHToNEUMat(const double Blh[], double Mat[]);
void CompSatElAz(const double Xre[], const double XrE[], const double Xs[], double *Elev, double *Azim); //���Ǹ߶ȽǷ�λ�Ǽ��㺯��
void Comp_dEnu(const double X0[], const double Xr[], double dENH[]);  //��λ�����㺯��

// NovAtel OEM7���ݽ��뺯��
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[],int flag);
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBS* obs);
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph);
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph);
int decode_psrpos(unsigned char* buff, EPOCHOBS* obs);
unsigned int crc32(const unsigned char* buff, int len);

// �㲥����
bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATMIDRES* Mid);
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);
void ComputeGPSSatOrbitAtSignalTrans(EPOCHOBS* Epk, GPSEPHREC* GpsEph, GPSEPHREC* BDSEph, double RcvPos[3]);
double hopfield(const double H, const double elev);

// SPP & SPV
void DetectOutlier(EPOCHOBS* Obs);  // �������̽��ֲ�
bool SPP(EPOCHOBS* Epoch, RAWDAT* Raw, PPRESULT* Result);
void SPV(EPOCHOBS* Epoch, PPRESULT* Result);

// RTK--LSQ
int GetSynObs(FILE* FBas, FILE* FRov, SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw);
void DetectLockTimeHalf(const EPOCHOBS* Obs, EPOCHOBS* CurObs);
void FormSDEpochObs(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs);
void DetectCycleSlip(SDEPOCHOBS* Obs);
void DetRefSat(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs);
void FormDDObs(const SDEPOCHOBS* SDObs, DDCOBS* DDObs);
void DDetectCycleSlip(DDCOBS* Obs);
void DetectHalf(const DDCOBS* Obs, DDCOBS* CurObs);
bool RTKFloat(RAWDAT* Raw, PPRESULT* Base, PPRESULT* Rov);
void RTKFixed(RAWDAT* Raw, double* RovPos, double* BasSat);

// RTK--EKF
bool InitEKF(DDCOBS* DDObs, double* RovPos, int& parnum, double* X, double* P);
void Prediction(DDCOBS* DDObs, DDCOBS* CurDDObs, int* Index, int& Curparnum, int& parnum, double* X0, double* X1, double* P0, double* P1);
bool DDUpdate(SDEPOCHOBS* CurSDObs, DDCOBS* CurDDObs, double* BasPos, EPOCHOBS* BasEpk, EPOCHOBS* RovEpk, int* Index, double* X1, double* P1);
bool RTKekf(RAWDAT* Raw, PPRESULT* Rov, RTKEKF* EKF);

// lambda
int LD(int n, const double* Q, double* L, double* D);
void gauss(int n, double* L, double* Z, int i, int j);
void perm(int n, double* L, double* D, int j, double del, double* Z);
void reduction(int n, double* L, double* D, double* Z);
int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s);
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);

// Socket����
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);
void CloseSocket(SOCKET& sock);

#endif
