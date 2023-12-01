//#define WIN32_LEAN_AND_MEAN

#include "SensorReader.h"
#include <stdio.h>
#include <cmath>
#include <ccMesh.h>
#include <ccGLMatrixTpl.h>
#include <mutex>

#define setbit(x, y) x |= (1 << y)     //置位
#define clrbit(x, y) x &= ~(1 << y)    //清除位
#define reversebit(x, y) x ^= (1 << y) //反转位
#define getbit(x, y) ((x) >> (y)&1)    //查看位

//Byte转有符号int 内联函数
int Byte2Int(byte &high, byte &low)
{
    int i = 1;
    if (high >= 128)
    {
        //遇到负数按位取反再 +1 ，变为它的相反数，最后再 x-1
        reversebit(high, 8);
        reversebit(high, 7);
        reversebit(high, 6);
        reversebit(high, 5);
        reversebit(high, 4);
        reversebit(high, 3);
        reversebit(high, 2);
        reversebit(high, 1);
        reversebit(low, 8);
        reversebit(low, 7);
        reversebit(low, 6);
        reversebit(low, 5);
        reversebit(low, 4);
        reversebit(low, 3);
        reversebit(low, 2);
        reversebit(low, 1);
        high += 1;
        low += 1;
        i = -1;
    }

    int t = (high << 8) | low;
    return t * i;
}

//数据包校验，c是byte数组，beginIdx是起始位置，默认校验11个数 内联函数
bool SbSumCheck(byte *c, unsigned int beginIdx)
{
    if (((c[beginIdx + 0] + c[beginIdx + 1] + c[beginIdx + 2] + c[beginIdx + 3] + c[beginIdx + 4] + c[beginIdx + 5] + c[beginIdx + 6] + c[beginIdx + 7] + c[beginIdx + 8] + c[beginIdx + 9]) & 0xff) == c[beginIdx + 10])
    {
        return true;
    }
    else
    {
        return false;
    }
}

SensorReader::SensorReader()
{
    // 初始化成员变量
    port = nullptr;
    receiveBuffer = new std::string("");
    working = false;
    t = nullptr;
    RunFlag = false;
    useNetWork = true;
    // 打开串口
    Open();
}

// 析构 - 必须安全地删除线程和释放串口资源
SensorReader::~SensorReader()
{
    Stop();
    Close();
}

// 打开串口
void SensorReader::Open()
{
    if (!useNetWork)
    {
        if (!port)
        {
            port = new serial::Serial(serialPort, baudRate, serial::Timeout::simpleTimeout(1000));
            port->close();
        }
        if (!(port->isOpen()))
        {
            port->open();
        }
    }
    else
    {
        socketkpr.Open();
    }
}

// 释放串口资源
void SensorReader::Close()
{
    if (!useNetWork)
    {
        port->close();
        delete port;
        port = nullptr;
    }
    //else
    //{
    //    socketkpr.Close();
    //}
}

// 向串口发送数据(不使用)
void SensorReader::Send(std::string &data)
{
    port->write(data);
}

// 接收数据
void SensorReader::ReceiveFormSerial()
{
    while (working)
    {
        //# 休眠一个微小的时间，可以避免无谓的CPU占用，在windows上实验时直接从12%下降到接近于0%
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        std::string tmp;
        //缓冲区有待接收的数据时
        if (port->available() > 0)
        {
            //读取数据
            int count = port->read(tmp, port->available());
            //string转byte
            const char *ta = tmp.c_str();
            char *b = new char[tmp.length() + 1];
            memset(b, 0, tmp.length() + 1);
            memcpy(b, ta, tmp.length());
            byte *c = (byte *)b; // byte 与 unsigned char 相同
            // 解析数据
            DecodeData(c);
            // 计算delta弧度
            if (RunFlag == false)
            {
                mtx.lock();
                rx = Angle[0];
                ry = Angle[1];
                rz = Angle[2];
                RunFlag = true;
                mtx.unlock();
            }
            else
            {
                mtx.lock();
                rx = (Angle[0] - lastrx);
                ry = (Angle[1] - lastry);
                rz = (Angle[2] - lastrz);
                mtx.unlock();
            }
        }
    }
}

void SensorReader::ReceiveFormNetWork()
{
    while (working)
    {
        //# 休眠一个微小的时间，可以避免无谓的CPU占用，在windows上实验时直接从12%下降到接近于0%
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //读取数据
        socketkpr.GetMsg(Data[8], Data[9], Data[10]);
        //# 姿态角 - rad 弧度
        Angle[0] = Data[8] / 32768.0 * M_PI;
        Angle[1] = Data[9] / 32768.0 * M_PI;
        Angle[2] = Data[10] / 32768.0 * M_PI;

        // 计算delta弧度
        if (RunFlag == false)
        {
            mtx.lock();
            rx = Angle[0];
            ry = Angle[1];
            rz = Angle[2];
            RunFlag = true;
            mtx.unlock();
        }
        else
        {
            mtx.lock();
            rx = (Angle[0] - lastrx);
            ry = (Angle[1] - lastry);
            rz = (Angle[2] - lastrz);
            mtx.unlock();
        }
    }
}

// 启动线程
void SensorReader::Start()
{
    std::cout << "NEW READER THREAD!\n";
    //# 开始数据读取线程
    if (t == nullptr)
    {
        working = true;
        if (!useNetWork)
        {
            t = new std::thread(&SensorReader::ReceiveFormSerial, this);
        }
        else
        {
            t = new std::thread(&SensorReader::ReceiveFormNetWork, this);
        }
    }
    else
    {
        working = true;
    }
}

// 停止线程
void SensorReader::Stop()
{
    working = false;
    if (t && t->joinable())
    {
        t->join();
    }
	//if (useNetWork) {
	//	socketkpr.Close();
	//}
}

// 屏幕输出(已弃用)
void SensorReader::Output()
{
    //	system("cls");//清屏
    //	printf("波特率：921600\n频率：200Hz\n\n");
    //	printf("x轴加速度：%.2f g\n", a[0]);
    //	printf("y轴加速度：%.2f g\n", a[1]);
    //	printf("z轴加速度：%.2f g\n\n", a[2]);
    //
    //	printf("x轴角速度：%.2f °/s\n", w[0]);
    //	printf("y轴角速度：%.2f °/s\n", w[1]);
    //	printf("z轴角速度：%.2f °/s\n\n", w[2]);
    //
    //	printf("x轴角度：%.2f °\n", Angle[0]);
    //	printf("y轴角度：%.2f °\n", Angle[1]);
    //	printf("z轴角度：%.2f °\n\n", Angle[2]);
    //
    //	printf("x轴磁场：%.0f mG\n", h[0]);
    //	printf("y轴磁场：%.0f mG\n", h[1]);
    //	printf("z轴磁场：%.0f mG\n\n", h[2]);
    //	printf("温   度：%.2f ℃\n", Temperature);
    //	std::cout << "============================" << std::endl << std::endl;
    //	/*printf("x轴速度：%.2f m/s\n", vx);
    //	printf("y轴速度：%.2f m/s\n", vy);
    //	printf("z轴速度：%.2f m/s\n", vz);*/
    //	//printf("v总速度：%.2f m/s\n\n", vall);
    //
    //	printf("v帧时间差：%.2f s\n\n", microseconds_difference);
    //	std::cout << "============================" << std::endl;
}

// 解析数据
void SensorReader::DecodeData(byte *c)
{
    //if (SbSumCheck(c, 0)) {
    //	Data[0] = Byte2Int(c[3], c[2]);//加速度x
    //	Data[1] = Byte2Int(c[5], c[4]);//加速度y
    //	Data[2] = Byte2Int(c[7], c[6]);//加速度z
    //	Data[3] = Byte2Int(c[9], c[8]);//温度 -10-11-12
    //	if (Data[3] / 100.0 <= 100) {
    //		Temperature = Data[3] / 100.0;
    //	}
    //	//# 加速度
    //	a[0] = Data[0] / 32768.0 * 16;
    //	a[1] = Data[1] / 32768.0 * 16;
    //	a[2] = Data[2] / 32768.0 * 16;
    //}
    //if (SbSumCheck(c, 11)) {
    //	Data[4] = Byte2Int(c[14], c[13]);//角速度x
    //	Data[5] = Byte2Int(c[16], c[15]);//角速度y
    //	Data[6] = Byte2Int(c[18], c[17]);//角速度z -19-20-21-22-23
    //	//# 角速度
    //	w[0] = Data[4] / 32768.0 * 200;
    //	w[1] = Data[5] / 32768.0 * 200;
    //	w[2] = Data[6] / 32768.0 * 200;
    //}
    if (SbSumCheck(c, 22))
    {
        Data[8] = Byte2Int(c[25], c[24]);  //姿态角x
        Data[9] = Byte2Int(c[27], c[26]);  //姿态角y
        Data[10] = Byte2Int(c[29], c[28]); //姿态角z -30-31-32-33-34
        //# 姿态角 - rad 弧度
        Angle[0] = Data[8] / 32768.0 * M_PI;
        Angle[1] = Data[9] / 32768.0 * M_PI;
        Angle[2] = Data[10] / 32768.0 * M_PI;
    }
    //if (SbSumCheck(c, 33)) {
    //	Data[12] = Byte2Int(c[36], c[35]);//磁场x
    //	Data[13] = Byte2Int(c[38], c[37]);//磁场y
    //	Data[14] = Byte2Int(c[40], c[39]);//磁场z
    //	//# 磁场
    //	h[0] = Data[12];
    //	h[1] = Data[13];
    //	h[2] = Data[14];
    //}
}
