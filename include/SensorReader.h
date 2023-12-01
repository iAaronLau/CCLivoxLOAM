#pragma once
#include "SocketKeeper.h"
#include <string>
#include <vector>
#include <thread>
#include <iostream>
#include "Serial.h"
#include <chrono>
#include <ctime>
#include <ccMesh.h>
#include <mutex>

using namespace std::chrono;

// ������������
const std::string serialPort = "COM3"; //# �˿ں�
const unsigned int baudRate = 921600;  //# ������
const unsigned int minPackageLen = 11; //# �ڴ����������У���С�İ�������11���ֽ�

// ����byte����
typedef unsigned char byte;

//# ���������ݶ�ȡ�� ����IMU���� �ײ�����
class SensorReader {
public:
	SensorReader();                  //# Ĭ�Ϲ��캯��
	~SensorReader();                 //# Ĭ���������� �ͷ���Դ
	void Open();					 //# ��
	void Close();					 //# �ر�
	void Send(std::string &data);    //# ��������
	void ReceiveFormSerial();		 //# ��������
	void ReceiveFormNetWork();		 //# ������������
	void Start();					 //# ��ʼ����
	void Stop();					 //# ֹͣ����
	void Output();					 //# ��Ļ��ӡ��Ϣ(������)

	double rx;						 // rotate-x ��̬�� - x
	double ry;						 // rotate-y ��̬�� - y
	double rz;						 // rotate-z ��̬�� - z
	//double a[4];
	//double w[4];
	//double h[4];
	//double Port[4];
	//double LastTime[10];
	//double Temperature;

private:
	void DecodeData(byte *byteTemp);// ��������
	double Data[16];                // ����������ݰ�
	double Angle[4];                // ��̬�� 
	double lastrx;                  // ��һ֡��̬�� - x
	double lastry;					// ��һ֡��̬�� - y
	double lastrz;					// ��һ֡��̬�� - z
	bool RunFlag;					// �Ƿ����й��ı�־
	bool working;					// �߳��Ƿ������еı�־
	bool useNetWork;				// �Ƿ�ʹ�����紫��IMU���ݵı�־
	std::thread *t;					// �̶߳����ָ��
	std::mutex mtx;					// �߳���mutex
	std::string *receiveBuffer;		// ���ݻ�����
	serial::Serial *port;			// �˿���Դ�����ָ��
	SocketKeeper socketkpr;         // IMU����ӿڶ���
};