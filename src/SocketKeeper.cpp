#include "SocketKeeper.h"

// 把char消息转为int/double - 内联函数
void str2int(char *str, int strSize, double &id1, double &id2, double &id3) {
	char d1[20]; memset(d1, 0, 20);
	char d2[20]; memset(d2, 0, 20);
	char d3[20]; memset(d3, 0, 20);
	unsigned int i = 0, j = 0;
	for (j = 0; i < strSize && str[i] != ' '; i++, j++) {
		d1[j] = str[i];
	}
	d1[j] = '\0';
	//i++;
	for (j = 0, i++; i < strSize && str[i] != ' '; i++, j++) {
		d2[j] = str[i];
	}
	d2[j] = '\0';
	//i++;
	for (j = 0, i++; i < strSize && str[i] != ' '; i++, j++) {
		d3[j] = str[i];
	}
	d3[j] = '\0';
	id1 = (double)atoi(d1);
	id2 = (double)atoi(d2);
	id3 = (double)atoi(d3);
}

// 默认构造函数 - 初始化成员变量
SocketKeeper::SocketKeeper() {
	// 初始换版本号
	wVersionRequested = MAKEWORD(2, 2);
	// 刷新缓冲区 - 初始化缓冲区
	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		return;
	}
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
		WSACleanup();
	}
}

// 默认析构函数 - 释放资源
SocketKeeper::~SocketKeeper() {
	Close();
}

// 打开端口 - port端口号 - type协议 UDP
void SocketKeeper::Open(const unsigned int port, const int type) {
	// 打开端口
	clientSock = socket(AF_INET, type, 0);
	if (INVALID_SOCKET == clientSock) {
		err = WSAGetLastError();
		WSACleanup();
	}

	// 未知意义
	sin.sin_family = AF_INET;
	sin.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	sin.sin_port = htons(port);

	// 开启端口监听
	if (bind(clientSock, (SOCKADDR*)&sin, sizeof(sin)) != 0) {
		err = WSAGetLastError();
		closesocket(clientSock);
		WSACleanup();
	}
	// 本机地址长度
	addrLen = sizeof(clientAddr);
	// 初始化recvBuff
	memset(recvBuff, 0, MAX_BUF_LEN);
}

// 关闭端口 - 释放端口资源
void SocketKeeper::Close() {
	closesocket(clientSock);
	WSACleanup();
}

// 从端口获取消息 - 三个参数分别是rotate-x y z
void SocketKeeper::GetMsg(double &data1, double &data2, double &data3) {
	// 收取信息 - 用recvBuff接收信息 - nSize消息长度
	nSize = recvfrom(clientSock, recvBuff, MAX_BUF_LEN, 0, (SOCKADDR*)&clientAddr, &addrLen);
	if (nSize == SOCKET_ERROR) {
		err = WSAGetLastError();
		return;
	}
	// 补全字符串的结尾
	recvBuff[nSize] = '\0';
	// 消息char转int/double 
	str2int(recvBuff, nSize, data1, data2, data3);
}
