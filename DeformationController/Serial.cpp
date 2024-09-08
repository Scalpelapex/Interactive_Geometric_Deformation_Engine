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
* 功能    ：	读串口线程回调函数
* 描述	   ：	收到数据后，简单的显示出来
********************************************************************************************/
//DWORD WINAPI CommProc(LPVOID lpParam) {
//
//	CSerial* pSerial = (CSerial*)lpParam;  //
//
//										   //清空串口
//	PurgeComm(pSerial->m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
//
//	char buf[512];
//	DWORD dwRead;
//	while (pSerial->m_hComm != INVALID_HANDLE_VALUE) {
//		BOOL bReadOK = ReadFile(pSerial->m_hComm, buf, 512, &dwRead, NULL);
//		if (bReadOK && (dwRead > 0)) {
//			buf[dwRead] = '\0';
//			//MessageBoxA(NULL, buf, "串口收到数据", MB_OK);
//			cout << "uart get:" << buf << endl;
//		}
//
//	}
//	return 0;
//}

/*******************************************************************************************
* 功能     ：	打开串口
* port     :	串口号, 如_T("COM1:")
* baud_rate:	波特率
* date_bits:	数据位（有效范围4~8）
* stop_bit :	停止位
* parity   :	奇偶校验。默认为无校验。NOPARITY 0； ODDPARITY 1；EVENPARITY 2；MARKPARITY 3；SPACEPARITY 4
********************************************************************************************/

BOOL CSerial::OpenSerialPort(const char* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity) {
	//打开串口
	m_hComm = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);//独占方式打开串口

	TCHAR err[512];

	if (m_hComm == INVALID_HANDLE_VALUE) {
		wsprintf(err, _T("打开串口%s 失败，请查看该串口是否已被占用"), port);
		MessageBox(NULL, err, _T("提示"), MB_OK);
		return FALSE;
	}

	//MessageBox(NULL,_T("打开成功"),_T("提示"),MB_OK);

	//获取串口默认配置
	DCB dcb;
	if (!GetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("获取串口当前属性参数失败"), _T("提示"), MB_OK);
	}

	//配置串口参数
	dcb.BaudRate = baud_rate;	//波特率
	dcb.fBinary = TRUE;			//二进制模式。必须为TRUE
	dcb.ByteSize = date_bits;	//数据位。范围4-8
	dcb.StopBits = ONESTOPBIT;	//停止位

	if (parity == NOPARITY) {
		dcb.fParity = FALSE;	//奇偶校验。无奇偶校验
		dcb.Parity = parity;	//校验模式。无奇偶校验
	}
	else {
		dcb.fParity = TRUE;		//奇偶校验。
		dcb.Parity = parity;	//校验模式。无奇偶校验
	}

	dcb.fOutxCtsFlow = FALSE;	//CTS线上的硬件握手
	dcb.fOutxDsrFlow = FALSE;	//DST线上的硬件握手
	dcb.fDtrControl = DTR_CONTROL_ENABLE; //DTR控制
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;//
	dcb.fOutX = FALSE;			//是否使用XON/XOFF协议
	dcb.fInX = FALSE;			//是否使用XON/XOFF协议
	dcb.fErrorChar = FALSE;		//是否使用发送错误协议
	dcb.fNull = FALSE;			//停用null stripping
	dcb.fRtsControl = RTS_CONTROL_ENABLE;//
	dcb.fAbortOnError = FALSE;	//串口发送错误，并不终止串口读写

								//设置串口参数
	if (!SetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("设置串口参数失败"), _T("提示"), MB_OK);
		return FALSE;
	}

	//设置串口事件
	SetCommMask(m_hComm, EV_RXCHAR); //在缓存中有字符时产生事件
	SetupComm(m_hComm, 16384, 16384);

	//设置串口读写时间
	COMMTIMEOUTS CommTimeOuts;
	GetCommTimeouts(m_hComm, &CommTimeOuts);
	CommTimeOuts.ReadIntervalTimeout = MAXDWORD;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 0;
	CommTimeOuts.WriteTotalTimeoutMultiplier = 10;
	CommTimeOuts.WriteTotalTimeoutConstant = 1000;

	if (!SetCommTimeouts(m_hComm, &CommTimeOuts)) {
		MessageBox(NULL, _T("设置串口时间失败"), _T("提示"), MB_OK);
		return FALSE;
	}

	//创建线程，读取数据
	//HANDLE hReadCommThread = (HANDLE)_beginthreadex(NULL, 0, (PTHREAD_START)CommProc, (LPVOID)this, 0, NULL);

	return TRUE;
}

/********************************************************************************************
* 功能    ：	通过串口发送一条数据
********************************************************************************************/
BOOL CSerial::SendData(char* data, int len) {
	if (m_hComm == INVALID_HANDLE_VALUE) {
		MessageBox(NULL, _T("串口未打开"), _T("提示"), MB_OK);
		return FALSE;
	}

	//清空串口
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	//写串口
	DWORD dwWrite = 0;
	DWORD dwRet = WriteFile(m_hComm, data, len, &dwWrite, NULL);

	//清空串口
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	if (!dwRet) {
		MessageBox(NULL, _T("发送数据失败"), _T("提示"), MB_OK);
		return FALSE;
	}
	return TRUE;
}

string CSerial::ReceiveData()
{
	std::string receivedData;

	if (m_hComm == INVALID_HANDLE_VALUE) {
		// 句柄无效，返回空字符串
		return receivedData;
	}

	char buf[512];
	DWORD dwRead;
	PurgeComm(m_hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);//清空缓冲区
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
		// 句柄无效，返回空字符串
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


//校验信息
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
		// 句柄无效，返回空字符串
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
		// 句柄无效，返回空字符串
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
		// 句柄无效，返回空字符串
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

