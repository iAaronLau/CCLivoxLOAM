#include "SocketKeeper.h"

// ��char��ϢתΪint/double - ��������
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

// Ĭ�Ϲ��캯�� - ��ʼ����Ա����
SocketKeeper::SocketKeeper() {
	// ��ʼ���汾��
	wVersionRequested = MAKEWORD(2, 2);
	// ˢ�»����� - ��ʼ��������
	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		return;
	}
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
		WSACleanup();
	}
}

// Ĭ���������� - �ͷ���Դ
SocketKeeper::~SocketKeeper() {
	Close();
}

// �򿪶˿� - port�˿ں� - typeЭ�� UDP
void SocketKeeper::Open(const unsigned int port, const int type) {
	// �򿪶˿�
	clientSock = socket(AF_INET, type, 0);
	if (INVALID_SOCKET == clientSock) {
		err = WSAGetLastError();
		WSACleanup();
	}

	// δ֪����
	sin.sin_family = AF_INET;
	sin.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	sin.sin_port = htons(port);

	// �����˿ڼ���
	if (bind(clientSock, (SOCKADDR*)&sin, sizeof(sin)) != 0) {
		err = WSAGetLastError();
		closesocket(clientSock);
		WSACleanup();
	}
	// ������ַ����
	addrLen = sizeof(clientAddr);
	// ��ʼ��recvBuff
	memset(recvBuff, 0, MAX_BUF_LEN);
}

// �رն˿� - �ͷŶ˿���Դ
void SocketKeeper::Close() {
	closesocket(clientSock);
	WSACleanup();
}

// �Ӷ˿ڻ�ȡ��Ϣ - ���������ֱ���rotate-x y z
void SocketKeeper::GetMsg(double &data1, double &data2, double &data3) {
	// ��ȡ��Ϣ - ��recvBuff������Ϣ - nSize��Ϣ����
	nSize = recvfrom(clientSock, recvBuff, MAX_BUF_LEN, 0, (SOCKADDR*)&clientAddr, &addrLen);
	if (nSize == SOCKET_ERROR) {
		err = WSAGetLastError();
		return;
	}
	// ��ȫ�ַ����Ľ�β
	recvBuff[nSize] = '\0';
	// ��Ϣcharתint/double 
	str2int(recvBuff, nSize, data1, data2, data3);
}
