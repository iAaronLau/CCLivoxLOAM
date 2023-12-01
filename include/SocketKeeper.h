#pragma once
#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>  
#include <iostream>  
#include <windows.h>  

#pragma comment(lib, "ws2_32.lib")  
// 监听端口
const unsigned int listenport = 9999;
// 接受数据包最大长度
const unsigned int MAX_BUF_LEN = 60;

using namespace std;

// IMU网络接口
class SocketKeeper {
public:
	SocketKeeper();
	~SocketKeeper();
	// 打开端口 - port端口号 - type协议 UDP
	void Open(const unsigned int port = listenport, const int type = SOCK_DGRAM);
	// 关闭端口
	void Close();
	// 从端口获取消息
	void GetMsg(double &data1, double &data2, double &data3);

private:
	WORD wVersionRequested;      // 底层版本信息
	WSADATA wsaData;             // 网口控制信息(硬件)
	SOCKET clientSock;           // socket端口
	SOCKADDR_IN sin;             // socket_in - 远程主机的socket
	SOCKADDR_IN clientAddr;      // 本机地址 - 本机的socket
	int err;                     // 错误状态 - 错误码
	int addrLen;                 // 本机地址长度
	int nSize;                   // 收到的消息长度
	char recvBuff[1024];         // 收到的消息(缓冲区)
};