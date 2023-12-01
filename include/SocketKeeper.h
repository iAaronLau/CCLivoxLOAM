#pragma once
#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>  
#include <iostream>  
#include <windows.h>  

#pragma comment(lib, "ws2_32.lib")  
// �����˿�
const unsigned int listenport = 9999;
// �������ݰ���󳤶�
const unsigned int MAX_BUF_LEN = 60;

using namespace std;

// IMU����ӿ�
class SocketKeeper {
public:
	SocketKeeper();
	~SocketKeeper();
	// �򿪶˿� - port�˿ں� - typeЭ�� UDP
	void Open(const unsigned int port = listenport, const int type = SOCK_DGRAM);
	// �رն˿�
	void Close();
	// �Ӷ˿ڻ�ȡ��Ϣ
	void GetMsg(double &data1, double &data2, double &data3);

private:
	WORD wVersionRequested;      // �ײ�汾��Ϣ
	WSADATA wsaData;             // ���ڿ�����Ϣ(Ӳ��)
	SOCKET clientSock;           // socket�˿�
	SOCKADDR_IN sin;             // socket_in - Զ��������socket
	SOCKADDR_IN clientAddr;      // ������ַ - ������socket
	int err;                     // ����״̬ - ������
	int addrLen;                 // ������ַ����
	int nSize;                   // �յ�����Ϣ����
	char recvBuff[1024];         // �յ�����Ϣ(������)
};