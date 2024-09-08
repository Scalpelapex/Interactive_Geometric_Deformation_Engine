#ifndef _SERIAL_H
#define _SERIAL_H


#include <windows.h>
#include <string>
#include <Eigen/Core>

using namespace std;

class CSerial
{
public:
	CSerial(void);
	~CSerial(void);

	//打开串口
	BOOL OpenSerialPort(const char* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity = NOPARITY);

	//发送数据
	BOOL SendData(char* data, int len);

	string ReceiveData();

	bool ConvertData(Eigen::MatrixXd &new_points, Eigen::MatrixXd &old_points);

public:
	HANDLE m_hComm;
};

bool JUDGE_DATA(const char* a);
void ANALYZE_DATA(const char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points);
void ANALYZE_DATA2(const char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points, double(&points)[18]);
bool Receive_Deformation(CSerial& serial, char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points);
bool Receive_Deformation2(CSerial& serial, char* buf, Eigen::MatrixXd& new_points, Eigen::MatrixXd& old_points, double(&points)[18]);
bool test(CSerial& serial, char* buf);



#endif
