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

// 常量参数定义
const std::string serialPort = "COM3"; //# 端口号
const unsigned int baudRate = 921600;  //# 波特率
const unsigned int minPackageLen = 11; //# 在传感器数据中，最小的包长度是11个字节

// 定义byte类型
typedef unsigned char byte;

//# 传感器数据读取类 处理IMU数据 底层驱动
class SensorReader {
public:
	SensorReader();                  //# 默认构造函数
	~SensorReader();                 //# 默认析构函数 释放资源
	void Open();					 //# 打开
	void Close();					 //# 关闭
	void Send(std::string &data);    //# 发送数据
	void ReceiveFormSerial();		 //# 接收数据
	void ReceiveFormNetWork();		 //# 接收网络数据
	void Start();					 //# 开始工作
	void Stop();					 //# 停止工作
	void Output();					 //# 屏幕打印信息(已弃用)

	double rx;						 // rotate-x 姿态角 - x
	double ry;						 // rotate-y 姿态角 - y
	double rz;						 // rotate-z 姿态角 - z
	//double a[4];
	//double w[4];
	//double h[4];
	//double Port[4];
	//double LastTime[10];
	//double Temperature;

private:
	void DecodeData(byte *byteTemp);// 解析数据
	double Data[16];                // 解析后的数据包
	double Angle[4];                // 姿态角 
	double lastrx;                  // 上一帧姿态角 - x
	double lastry;					// 上一帧姿态角 - y
	double lastrz;					// 上一帧姿态角 - z
	bool RunFlag;					// 是否运行过的标志
	bool working;					// 线程是否在运行的标志
	bool useNetWork;				// 是否使用网络传输IMU数据的标志
	std::thread *t;					// 线程对象的指针
	std::mutex mtx;					// 线程锁mutex
	std::string *receiveBuffer;		// 数据缓冲区
	serial::Serial *port;			// 端口资源对象的指针
	SocketKeeper socketkpr;         // IMU网络接口对象
};