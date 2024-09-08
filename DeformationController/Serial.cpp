#include <TCHAR.h>
#include "Serial.h"
#include <process.h>
#include <string.h>
#include <iostream>

using namespace std;
typedef unsigned(__stdcall* PTHREAD_START) (void*);

CSerial::CSerial(void)
{
	m_hComm = INVALID_HANDLE_VALUE;
}

CSerial::~CSerial(void)
{
	if (m_hComm != INVALID_HANDLE_VALUE) {
		CloseHandle(m_hComm);
	}
}

/*********************************************************************************************
* ����    ��	�������̻߳ص�����
* ����	   ��	�յ����ݺ󣬼򵥵���ʾ����
********************************************************************************************/
//DWORD WINAPI CommProc(LPVOID lpParam) {
//
//	CSerial* pSerial = (CSerial*)lpParam;  //
//
//										   //��մ���
//	PurgeComm(pSerial->m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
//
//	char buf[512];
//	DWORD dwRead;
//	while (pSerial->m_hComm != INVALID_HANDLE_VALUE) {
//		BOOL bReadOK = ReadFile(pSerial->m_hComm, buf, 512, &dwRead, NULL);
//		if (bReadOK && (dwRead > 0)) {
//			buf[dwRead] = '\0';
//			//MessageBoxA(NULL, buf, "�����յ�����", MB_OK);
//			cout << "uart get:" << buf << endl;
//		}
//
//	}
//	return 0;
//}

/*******************************************************************************************
* ����     ��	�򿪴���
* port     :	���ں�, ��_T("COM1:")
* baud_rate:	������
* date_bits:	����λ����Ч��Χ4~8��
* stop_bit :	ֹͣλ
* parity   :	��żУ�顣Ĭ��Ϊ��У�顣NOPARITY 0�� ODDPARITY 1��EVENPARITY 2��MARKPARITY 3��SPACEPARITY 4
********************************************************************************************/

BOOL CSerial::OpenSerialPort(const char* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity) {
	//�򿪴���
	m_hComm = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);//��ռ��ʽ�򿪴���

	TCHAR err[512];

	if (m_hComm == INVALID_HANDLE_VALUE) {
		wsprintf(err, _T("�򿪴���%s ʧ�ܣ���鿴�ô����Ƿ��ѱ�ռ��"), port);
		MessageBox(NULL, err, _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//MessageBox(NULL,_T("�򿪳ɹ�"),_T("��ʾ"),MB_OK);

	//��ȡ����Ĭ������
	DCB dcb;
	if (!GetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("��ȡ���ڵ�ǰ���Բ���ʧ��"), _T("��ʾ"), MB_OK);
	}

	//���ô��ڲ���
	dcb.BaudRate = baud_rate;	//������
	dcb.fBinary = TRUE;			//������ģʽ������ΪTRUE
	dcb.ByteSize = date_bits;	//����λ����Χ4-8
	dcb.StopBits = ONESTOPBIT;	//ֹͣλ

	if (parity == NOPARITY) {
		dcb.fParity = FALSE;	//��żУ�顣����żУ��
		dcb.Parity = parity;	//У��ģʽ������żУ��
	}
	else {
		dcb.fParity = TRUE;		//��żУ�顣
		dcb.Parity = parity;	//У��ģʽ������żУ��
	}

	dcb.fOutxCtsFlow = FALSE;	//CTS���ϵ�Ӳ������
	dcb.fOutxDsrFlow = FALSE;	//DST���ϵ�Ӳ������
	dcb.fDtrControl = DTR_CONTROL_ENABLE; //DTR����
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;//
	dcb.fOutX = FALSE;			//�Ƿ�ʹ��XON/XOFFЭ��
	dcb.fInX = FALSE;			//�Ƿ�ʹ��XON/XOFFЭ��
	dcb.fErrorChar = FALSE;		//�Ƿ�ʹ�÷��ʹ���Э��
	dcb.fNull = FALSE;			//ͣ��null stripping
	dcb.fRtsControl = RTS_CONTROL_ENABLE;//
	dcb.fAbortOnError = FALSE;	//���ڷ��ʹ��󣬲�����ֹ���ڶ�д

								//���ô��ڲ���
	if (!SetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("���ô��ڲ���ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//���ô����¼�
	SetCommMask(m_hComm, EV_RXCHAR); //�ڻ��������ַ�ʱ�����¼�
	SetupComm(m_hComm, 16384, 16384);

	//���ô��ڶ�дʱ��
	COMMTIMEOUTS CommTimeOuts;
	GetCommTimeouts(m_hComm, &CommTimeOuts);
	CommTimeOuts.ReadIntervalTimeout = MAXDWORD;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 0;
	CommTimeOuts.WriteTotalTimeoutMultiplier = 10;
	CommTimeOuts.WriteTotalTimeoutConstant = 1000;

	if (!SetCommTimeouts(m_hComm, &CommTimeOuts)) {
		MessageBox(NULL, _T("���ô���ʱ��ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//�����̣߳���ȡ����
	//HANDLE hReadCommThread = (HANDLE)_beginthreadex(NULL, 0, (PTHREAD_START)CommProc, (LPVOID)this, 0, NULL);

	return TRUE;
}

/********************************************************************************************
* ����    ��	ͨ�����ڷ���һ������
********************************************************************************************/
BOOL CSerial::SendData(char* data, int len) {
	if (m_hComm == INVALID_HANDLE_VALUE) {
		MessageBox(NULL, _T("����δ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//��մ���
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	//д����
	DWORD dwWrite = 0;
	DWORD dwRet = WriteFile(m_hComm, data, len, &dwWrite, NULL);

	//��մ���
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	if (!dwRet) {
		MessageBox(NULL, _T("��������ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}
	return TRUE;
}

string CSerial::ReceiveData()
{
	std::string receivedData;

	if (m_hComm == INVALID_HANDLE_VALUE) {
		// �����Ч�����ؿ��ַ���
		return receivedData;
	}

	char buf[512];
	DWORD dwRead;
	PurgeComm(m_hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����
	BOOL bReadStat = ReadFile(m_hComm,
		buf,
		sizeof(buf) - 1,
		&dwRead,
		NULL);

	if (bReadStat && dwRead > 0) {
		buf[dwRead] = '\0';
		receivedData += buf;
	}

	return receivedData;

}

bool CSerial::ConvertData(Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points)
{
	std::string receivedData;

	if (m_hComm == INVALID_HANDLE_VALUE) {
		// �����Ч�����ؿ��ַ���
		return false;
	}
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
	char buf[512];
	DWORD dwRead;
	if (m_hComm != INVALID_HANDLE_VALUE) 
	{
		BOOL bReadOK = ReadFile(m_hComm, buf, 512, &dwRead, NULL);
		if (bReadOK && (dwRead > 0)) 
		{
			buf[dwRead] = '\0';
			if (JUDGE_DATA(buf))
			{
				cout << "uart get:" << buf << endl;
				ANALYZE_DATA(buf, new_points, old_points);
				return true;
			}
			else
			{
				cout << "!!!!! - - Wrong Data - - !!!!!\n" << buf << endl;
				return false;
			}
		}
		else
		{
			return false;
		}
	}
}


//У����Ϣ
bool JUDGE_DATA(const char* a)
{
	if (a[0] == 'N' && a[1] > '0')
	{
		int n = a[1] - '0';
		for (int i = 0; i < n; i++)
		{
			if (a[2 + i * 18] == 'X' && a[8 + i * 18] == 'Y' && a[14 + i * 18] == 'Z'){}
			else
			{
				return false;
			}
		}
		return true;
	}
	return false;
}

void ANALYZE_DATA(const char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points)
{
	int num_points = buf[1] - '0';
	Eigen::MatrixXd temp_old(num_points, 3);
	Eigen::MatrixXd temp_new(num_points, 3);
	for (int i = 0; i < num_points; i++)
	{
		int as = buf[3 + i * 36] - '0';
		int a0 = buf[4 + i * 36] - '0';
		int a1 = buf[5 + i * 36] - '0';
		int a2 = buf[6 + i * 36] - '0';
		int a3 = buf[7 + i * 36] - '0';
		double xp = a0 * 10.0 + a1 + a2 * 0.1 + a3 * 0.01;
		int b0 = buf[10 + i * 36] - '0';
		int b1 = buf[11 + i * 36] - '0';
		int b2 = buf[12 + i * 36] - '0';
		int b3 = buf[13 + i * 36] - '0';
		double yp = b0 * 10.0 + b1 + b2 * 0.1 + b3 * 0.01;
		int c0 = buf[16 + i * 36] - '0';
		int c1 = buf[17 + i * 36] - '0';
		int c2 = buf[18 + i * 36] - '0';
		int c3 = buf[19 + i * 36] - '0';
		double zp = c0 * 10.0 + c1 + c2 * 0.1 + c3 * 0.01;
		if (buf[3  + i * 36] == '1') { xp = -xp; }
		if (buf[9  + i * 36] == '1') { yp = -yp; }
		if (buf[15 + i * 36] == '1') { zp = -zp; }
		temp_old(i, 0) = xp;
		temp_old(i, 1) = yp;
		temp_old(i, 2) = zp;

		int a00 = buf[22 + i * 36] - '0';
		int a11 = buf[23 + i * 36] - '0';
		int a22 = buf[24 + i * 36] - '0';
		int a33 = buf[25 + i * 36] - '0';
		double xn = a00 * 10.0 + a11 + a22 * 0.1 + a33 * 0.01;
		int b00 = buf[28 + i * 36] - '0';
		int b11 = buf[29 + i * 36] - '0';
		int b22 = buf[30 + i * 36] - '0';
		int b33 = buf[31 + i * 36] - '0';
		double yn = b00 * 10.0 + b11 + b22 * 0.1 + b33 * 0.01;
		int c00 = buf[34 + i * 36] - '0';
		int c11 = buf[35 + i * 36] - '0';
		int c22 = buf[36 + i * 36] - '0';
		int c33 = buf[37 + i * 36] - '0';
		double zn = c00 * 10.0 + c11 + c22 * 0.1 + c33 * 0.01;
		if (buf[21 + i * 36] == '1') { xn = -xn; }
		if (buf[27 + i * 36] == '1') { yn = -yn; }
		if (buf[33 + i * 36] == '1') { zn = -zn; }
		temp_new(i, 0) = xn;
		temp_new(i, 1) = yn;
		temp_new(i, 2) = zn;
	}
	new_points = temp_new;
	old_points = temp_old;
	return;
}

void ANALYZE_DATA2(const char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points, double (&points)[18])
{
	Eigen::MatrixXd temp_old(6, 3);
	Eigen::MatrixXd temp_new(6, 3);
	
	for (int i = 0; i < 6; i++)
	{
		temp_old(i, 0) = points[3 * i + 0];
		temp_old(i, 1) = points[3 * i + 1];
		temp_old(i, 2) = points[3 * i + 2];
	}

	if (buf[1] == '1') { points[1] += 0.1; }
	if (buf[1] == '0') { points[1] -= 0.1; }
	
	for (int i = 0; i < 6; i++)
	{
		temp_new(i, 0) = points[3 * i + 0];
		temp_new(i, 1) = points[3 * i + 1];
		temp_new(i, 2) = points[3 * i + 2];
	}
	new_points = temp_new;
	old_points = temp_old;
	return;
}


bool Receive_Deformation(CSerial& serial, char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points)
{
	if (serial.m_hComm == INVALID_HANDLE_VALUE) {
		// �����Ч�����ؿ��ַ���
		return false;
	}
	//PurgeComm(serial.m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	DWORD dwRead;
	if (serial.m_hComm != INVALID_HANDLE_VALUE)
	{
		BOOL bReadOK = ReadFile(serial.m_hComm, buf, 512, &dwRead, NULL);
		//cout << dwRead << endl;
		/*if (buf[1])
		{
			cout << dwRead << endl;
			cout << buf[1] << endl;
		}*/
		///  -- - - - BUG
		if (bReadOK && (dwRead > 0))
		{
			buf[dwRead] = '\0';
			cout << buf[1] << endl;
			
			if (JUDGE_DATA(buf))
			{
				cout << "uart get:" << buf << endl;
				ANALYZE_DATA(buf, new_points, old_points);
				return true;
			}
			else
			{
				cout << "!!!!! - - Wrong Data - - !!!!!\n" << buf << endl;
				return false;
			}
		}
		else
		{
			return false;
		}
	}

}

bool Receive_Deformation2(CSerial& serial, char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points, double (&points)[18])
{
	if (serial.m_hComm == INVALID_HANDLE_VALUE) {
		// �����Ч�����ؿ��ַ���
		return false;
	}
	DWORD dwRead;
	if (serial.m_hComm != INVALID_HANDLE_VALUE)
	{
		BOOL bReadOK = ReadFile(serial.m_hComm, buf, 512, &dwRead, NULL);
		if (bReadOK && (dwRead > 0))
		{
			buf[dwRead] = '\0';
			cout << buf[1] << endl;

			if (buf[0]='X')
			{
				cout << "uart get:" << buf << endl;
				ANALYZE_DATA2(buf, new_points, old_points, points);
				return true;
			}
			else
			{
				cout << "!!!!! - - Wrong Data - - !!!!!\n" << buf << endl;
				return false;
			}
		}
		else
		{
			return false;
		}
	}
}


bool test(CSerial& serial, char* buf)
{
	if (serial.m_hComm == INVALID_HANDLE_VALUE) {
		// �����Ч�����ؿ��ַ���
		return false;
	}
	
	DWORD dwRead;
	BOOL bReadStat = ReadFile(serial.m_hComm,
		buf,
		sizeof(buf) - 1,
		&dwRead,
		NULL);
	if (bReadStat && dwRead > 0) {
		buf[dwRead] = '\0';
		cout << buf[1] << endl;;
		return true;
	}
	return false;
}

