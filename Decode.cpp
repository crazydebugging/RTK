// Decode.cpp : ���ļ�����NovAtel OEM7���ݽ��뺯����
//
#include "RTK_Structs.h"

#define OBSERVATION_DATA_TYPE 43
#define GPS_EPHEMERIS_TYPE 7
#define BDS_EPHEMERIS_TYPE 1696
#define POSITION_RESULT_TYPE 42
#define DATA_HEADER_SIZE 28
#define CRC_SIZE 4

#define POLYCRC32 0xEDB88320u /* CRC32 polynomial */

double D8(unsigned char* p)
{
    double r;
    memcpy(&r, p, 8);
    return r;
}
float F4(unsigned char* p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}
unsigned short Us2(unsigned char* p)
{
    unsigned short r;
    memcpy(&r, p, 2);
    return r;
}
short S2(unsigned char* p)
{
    short r;
    memcpy(&r, p, 2);
    return r;
}
bool B1(unsigned char* p)
{
    bool r;
    memcpy(&r, p, 1);
    return r;
}
int I4(unsigned char* p)
{
    int r;
    memcpy(&r, p, 4);
    return r;
}
unsigned int Ui4(unsigned char* p)
{
    unsigned int r;
    memcpy(&r, p, 4);
    return r;
}
unsigned long UI4(unsigned char* p)
{
    unsigned long r;
    memcpy(&r, p, 4);
    return r;
}
unsigned char Uc1(unsigned char* p)
{
    unsigned char r;
    memcpy(&r, p, 1);
    return r;
}

/****************************************************************************
  DecodeNovOem7Dat

  Ŀ�ģ�����NovAtel OEM7���ݽ���������

  ��ţ�301

  ����:
  Buff[]		��������ݴ�
  len			������Ч�ֽ��������ؾ�������������ַ�����
  obs		    ����õ��Ĺ۲�ֵ����
  geph		    ����õ���gps��������
  beph		    ����õ���bps��������
  flag          �ж��Ǵ���ʵʱ0�����ļ�1

  ����ֵ������ʧ�� 0���ɽ����� 1-4,�޿ɽ�۲�ֵ���5
****************************************************************************/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[],int flag)
{
    int result = 5;
    int offset = 0;

    int temp1 = len;
    while (offset < len) {
        //���ҵ�ͷ
        int i = 0;
        while (buff[offset+i] != (unsigned char) 0xAA || buff[offset+i+1] != (unsigned char)0x44 || buff[offset+i+2] != (unsigned char)0x12)
        {
            i++;
            // ��� offset �Ƿ񳬹� len
            if (offset+i+2 >= len) {
                break; // ����ѭ��
            }
        }
        offset = offset + i;
        //��ȡ����ͷheader
        // ��� offset �Ƿ񳬹� len
        if (offset + DATA_HEADER_SIZE >= len) {
            break; // ����ѭ��
        }
        // �ҵ����������Լ��м����ݵĳ��ȣ��������������ͽ��н���
        unsigned short MessageID = Us2(buff+offset+4);
        unsigned short MessageLength = Us2(buff + offset + 8);
        int Length = int(MessageLength+ DATA_HEADER_SIZE+ CRC_SIZE);
        //�ж������ܷ�ͨ��CRC����,��ɾ��������־��Ϣ
        // ��� offset �Ƿ񳬹� len
        if (offset + Length >= len) {
            break; // ����ѭ��
        }
        if (Ui4(buff + offset + Length - CRC_SIZE) != crc32(buff + offset, Length - CRC_SIZE))
        {
            // ����ƫ����
            offset += Length;
            continue;
        }
        //��ԪGPS
        unsigned short Week = Us2(buff +offset+ 14);
        double SecOfWeek = 0.001 * I4(buff +offset+ 16);
        // ����۲�ֵ����
        if (MessageID == OBSERVATION_DATA_TYPE) {
            obs->Time.Week = Week;
            obs->Time.SecOfWeek = SecOfWeek;
            memset(&obs->SatObs, 0, sizeof(obs->SatObs));// ʹ��memset()��սṹ���еĹ۲�����
            decode_rangeb_oem7(buff + offset,obs);
            result = 1; // �۲�ֵ���
            // ����ƫ����
            offset += Length;
            break;
         }
        // �����������͵�����
        else if (MessageID == GPS_EPHEMERIS_TYPE) {
            decode_gpsephem(buff + offset, geph);
            result = 2;// GPS�������
        }
        else if (MessageID == BDS_EPHEMERIS_TYPE) {
            decode_bdsephem(buff + offset, beph);
            result = 3;// BDS�������
        }
        else if (MessageID == POSITION_RESULT_TYPE) {
            decode_psrpos(buff + offset, obs);
            result = 4;// ���ջ������� 
        }
        // ����ƫ����
        offset += Length;
    }
    // ����ʣ���ֽڳ���
    len -= offset;

    // ��δ������ֽڸ��Ƶ�buff����λ��
    if (len > 0) {
        if (flag==1)
        {
            memmove(buff, buff + MAXRAWLEN - len, len);
            // ��� buff ��ʣ�µ�����
            memset(buff + len, 0, MAXRAWLEN - len);
        }
        else
        {
            memmove(buff, buff + temp1 - len, len);
            // ��� buff ��ʣ�µ�����
            memset(buff + len, 0, 3 * MAXRAWLEN - len);
        }
    }
    return result;
}

/****************************************************************************
  decode_rangeb_oem7

  Ŀ�ģ��������ݽ������۲�ֵ�Ӻ���

  ��ţ�302

  ����:
  buff[]		��������ݴ�
  obs		    ����õ��Ĺ۲�ֵ����

  ����ֵ������ʧ�� 0������ɹ� 1
****************************************************************************/
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBS* obs)
{
    unsigned long obssum = UI4(buff + DATA_HEADER_SIZE);
    unsigned short obscount = 0;
    unsigned short satnum = 0;
    while(obscount<obssum)
    { 
        unsigned char* p = buff + DATA_HEADER_SIZE + obscount * 44;
        unsigned long status = UI4(p+ 44);
        int f = 0;//Ƶ�κ�
        GNSSSys sys = UNKS;//����ϵͳ
        int n = 0;//��ǰ���Ǻ��±�
        double wl;//�ز�����
        switch ((status >> 16) & 0x7)
        {
        case 0:sys = GPS;
            switch ((status >> 21) & 0x1F)
             {
            case 0: f = 0; wl = WL1_GPS; break;
             case 9: f = 1; wl = WL2_GPS; break;
             default: f = 2; break;
             }
            break;
        case 4:sys = BDS;
            switch ((status >> 21) & 0x1F)
            {
            case 0:
            case 4:
                f = 0;  wl = WL1_BDS; break;
            case 2: 
            case 6: 
                f = 1; wl = WL3_BDS; break;
            default: f = 2; break;
            }
            break;
        default:sys = UNKS;
        }
        if (sys == UNKS || f == 2) {
            obscount++; continue;
        }
        unsigned short prn = Us2(p + 4);
        
        //��ǰ�۲�����Ǻź�ϵͳ�����Ѿ�������������Ƿ���ڣ����ھͷ��ص�ǰ�������±�
        //��������ڣ��ҵ���һ����0�������±�
        for (int j = 0; j < MAXCHANNUM; j++)
        {
            if (obs->SatObs[j].Prn == prn && obs->SatObs[j].System == sys)
            {
                n = j;
                break;
            }
            if (obs->SatObs[j].Prn == 0 && obs->SatObs[j].System == UNKS)
            {
                n = j;
                break;
            }
        }
        obs->SatObs[n].Prn = prn;
        obs->SatObs[n].System = sys;

        obs->SatObs[n].P[f] = D8(p + 8);
        obs->SatObs[n].L[f] = -D8(p + 20)*wl;
        obs->SatObs[n].D[f] = -F4(p + 32) * wl;
        obs->SatObs[n].cn0[f] = F4(p + 36);
        obs->SatObs[n].LockTime[f] = F4(p + 40);
        obs->SatObs[n].half[f] = (status >> 11) & 0x1;
        satnum = (satnum > n) ? satnum : n;
        obscount++;
    }
    obs->SatNum = satnum + 1;
    return 1;
}
/****************************************************************************
  decode_gpsephem

  Ŀ�ģ��������ݽ������GPS�����Ӻ���

  ��ţ�303

  ����:
  Buff[]		��������ݴ�
  eph		    ����õ���gps��������
  ����ֵ������ʧ�� 0������ɹ� 1
****************************************************************************/
int decode_gpsephem(unsigned char* Buff, GPSEPHREC* eph)
{
    unsigned char* buff = Buff + DATA_HEADER_SIZE;
    unsigned short prn = UI4(buff);
    if (prn > 32 || prn < 1)    return 0;
    GPSEPHREC* gpseph;
    gpseph = eph + prn - 1;
    gpseph->PRN = prn;
    gpseph->SVHealth = UI4(buff+12);
    gpseph->IODE = UI4(buff+20);
    gpseph->TOC.Week = UI4(buff+24);
     gpseph->TOE.Week = UI4(buff+24);
     gpseph->TOE.SecOfWeek = D8(buff+32);
     gpseph->SqrtA = sqrt(D8(buff+40));
     gpseph->DeltaN = D8(buff+48);
     gpseph->M0 = D8(buff+56);
     gpseph->e = D8(buff+64);
     gpseph->omega = D8(buff+72);
     gpseph->Cuc = D8(buff+80);
     gpseph->Cus = D8(buff + 88);
     gpseph->Crc = D8(buff + 96);
     gpseph->Crs = D8(buff + 104);
     gpseph->Cic = D8(buff + 112);
     gpseph->Cis = D8(buff + 120);
     gpseph->i0 = D8(buff + 128);
     gpseph->iDot = D8(buff + 136);
     gpseph->OMEGA = D8(buff + 144);
     gpseph->OMEGADot = D8(buff + 152);
     gpseph->IODC = UI4(buff + 160);
     gpseph->TOC.SecOfWeek = D8(buff + 164);
     gpseph->TGD1 = D8(buff + 172);
     gpseph->ClkBias = D8(buff + 180);
     gpseph->ClkDrift = D8(buff + 188);
     gpseph->ClkDriftRate = D8(buff + 196);
     gpseph->Sys = GPS;
    return 1;
}
/****************************************************************************
  decode_bdsephem

  Ŀ�ģ��������ݽ������BDS�����Ӻ���

  ��ţ�304

  ����:
  Buff[]		��������ݴ�
  eph		    ����õ���bps��������

  ����ֵ������ʧ�� 0������ɹ� 1
****************************************************************************/
int decode_bdsephem(unsigned char* Buff, GPSEPHREC* eph)
{
    unsigned char* buff = Buff + DATA_HEADER_SIZE;
    unsigned short prn = UI4(buff);
    if (prn > 63 || prn < 1)    return 0;
    GPSEPHREC* bdseph;
    bdseph = eph + prn - 1;
    bdseph->PRN = prn;
    bdseph->SVHealth = UI4(buff  + 16);
    bdseph->TOC.Week = UI4(buff  + 4);
    bdseph->TOE.Week = UI4(buff  + 4);
    bdseph->TOE.SecOfWeek = UI4(buff  + 72);
    bdseph->SqrtA = D8(buff  + 76);
    bdseph->DeltaN = D8(buff  + 100);
    bdseph->M0 = D8(buff  + 108);
    bdseph->e = D8(buff  + 84);
    bdseph->omega = D8(buff  + 92);
    bdseph->Cuc = D8(buff  + 148);
    bdseph->Cus = D8(buff  + 156);
    bdseph->Crc = D8(buff  + 164);
    bdseph->Crs = D8(buff  + 172);
    bdseph->Cic = D8(buff  + 180);
    bdseph->Cis = D8(buff  + 188);
    bdseph->i0 = D8(buff  + 132);
    bdseph->iDot = D8(buff  + 140);
    bdseph->OMEGA = D8(buff  + 116);
    bdseph->OMEGADot = D8(buff  + 124);
    bdseph->TOC.SecOfWeek = UI4(buff  + 40);
    bdseph->TGD1 = D8(buff  + 20);
    bdseph->TGD2 = D8(buff  + 28);
    bdseph->ClkBias = D8(buff  + 44);
    bdseph->ClkDrift = D8(buff  + 52);
    bdseph->ClkDriftRate = D8(buff  + 60);
    bdseph->Sys = BDS;
    return 1;
}
/****************************************************************************
  decode_psrpos

  Ŀ�ģ��������ݽ������POS����Ӻ���

  ��ţ�305

  ����:
  Buff[]		��������ݴ�
  obs		    ����õ��Ľ��ջ�λ������

  ����ֵ������ʧ�� 0������ɹ� 1
****************************************************************************/
int decode_psrpos(unsigned char* Buff, EPOCHOBS* obs)
{
    unsigned char* buff = Buff + DATA_HEADER_SIZE;
    double Pos[3] = { 0 };
    double xyz[3] = { 0 };

    Pos[0] = D8(buff + 8);
    Pos[1] = D8(buff + 16);
    Pos[2] = D8(buff + 24);
    Pos[2] += F4(buff + 32);

    Pos[0] = Pos[0] * Rad; Pos[1] = Pos[1] * Rad;
    BLHToXYZ(Pos, xyz, R_WGS84, F_WGS84);
    obs->Pos[0] = xyz[0];
    obs->Pos[1] = xyz[1];
    obs->Pos[2] = xyz[2];
    return 1;
}
/****************************************************************************
  crc32

  Ŀ�ģ���������У��

  ��ţ�306

  ����:
  buff		��������ݴ�
  len		��Ҫ����ĳ���

  ����ֵ������crc
****************************************************************************/
unsigned int crc32(const unsigned char* buff, int len)
{
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++)
    {
        crc ^= buff[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}