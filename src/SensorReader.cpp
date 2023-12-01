//#define WIN32_LEAN_AND_MEAN

#include "SensorReader.h"
#include <stdio.h>
#include <cmath>
#include <ccMesh.h>
#include <ccGLMatrixTpl.h>
#include <mutex>

#define setbit(x, y) x |= (1 << y)     //��λ
#define clrbit(x, y) x &= ~(1 << y)    //���λ
#define reversebit(x, y) x ^= (1 << y) //��תλ
#define getbit(x, y) ((x) >> (y)&1)    //�鿴λ

//Byteת�з���int ��������
int Byte2Int(byte &high, byte &low)
{
    int i = 1;
    if (high >= 128)
    {
        //����������λȡ���� +1 ����Ϊ�����෴��������� x-1
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

//���ݰ�У�飬c��byte���飬beginIdx����ʼλ�ã�Ĭ��У��11���� ��������
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
    // ��ʼ����Ա����
    port = nullptr;
    receiveBuffer = new std::string("");
    working = false;
    t = nullptr;
    RunFlag = false;
    useNetWork = true;
    // �򿪴���
    Open();
}

// ���� - ���밲ȫ��ɾ���̺߳��ͷŴ�����Դ
SensorReader::~SensorReader()
{
    Stop();
    Close();
}

// �򿪴���
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

// �ͷŴ�����Դ
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

// �򴮿ڷ�������(��ʹ��)
void SensorReader::Send(std::string &data)
{
    port->write(data);
}

// ��������
void SensorReader::ReceiveFormSerial()
{
    while (working)
    {
        //# ����һ��΢С��ʱ�䣬���Ա�����ν��CPUռ�ã���windows��ʵ��ʱֱ�Ӵ�12%�½����ӽ���0%
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        std::string tmp;
        //�������д����յ�����ʱ
        if (port->available() > 0)
        {
            //��ȡ����
            int count = port->read(tmp, port->available());
            //stringתbyte
            const char *ta = tmp.c_str();
            char *b = new char[tmp.length() + 1];
            memset(b, 0, tmp.length() + 1);
            memcpy(b, ta, tmp.length());
            byte *c = (byte *)b; // byte �� unsigned char ��ͬ
            // ��������
            DecodeData(c);
            // ����delta����
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
        //# ����һ��΢С��ʱ�䣬���Ա�����ν��CPUռ�ã���windows��ʵ��ʱֱ�Ӵ�12%�½����ӽ���0%
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //��ȡ����
        socketkpr.GetMsg(Data[8], Data[9], Data[10]);
        //# ��̬�� - rad ����
        Angle[0] = Data[8] / 32768.0 * M_PI;
        Angle[1] = Data[9] / 32768.0 * M_PI;
        Angle[2] = Data[10] / 32768.0 * M_PI;

        // ����delta����
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

// �����߳�
void SensorReader::Start()
{
    std::cout << "NEW READER THREAD!\n";
    //# ��ʼ���ݶ�ȡ�߳�
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

// ֹͣ�߳�
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

// ��Ļ���(������)
void SensorReader::Output()
{
    //	system("cls");//����
    //	printf("�����ʣ�921600\nƵ�ʣ�200Hz\n\n");
    //	printf("x����ٶȣ�%.2f g\n", a[0]);
    //	printf("y����ٶȣ�%.2f g\n", a[1]);
    //	printf("z����ٶȣ�%.2f g\n\n", a[2]);
    //
    //	printf("x����ٶȣ�%.2f ��/s\n", w[0]);
    //	printf("y����ٶȣ�%.2f ��/s\n", w[1]);
    //	printf("z����ٶȣ�%.2f ��/s\n\n", w[2]);
    //
    //	printf("x��Ƕȣ�%.2f ��\n", Angle[0]);
    //	printf("y��Ƕȣ�%.2f ��\n", Angle[1]);
    //	printf("z��Ƕȣ�%.2f ��\n\n", Angle[2]);
    //
    //	printf("x��ų���%.0f mG\n", h[0]);
    //	printf("y��ų���%.0f mG\n", h[1]);
    //	printf("z��ų���%.0f mG\n\n", h[2]);
    //	printf("��   �ȣ�%.2f ��\n", Temperature);
    //	std::cout << "============================" << std::endl << std::endl;
    //	/*printf("x���ٶȣ�%.2f m/s\n", vx);
    //	printf("y���ٶȣ�%.2f m/s\n", vy);
    //	printf("z���ٶȣ�%.2f m/s\n", vz);*/
    //	//printf("v���ٶȣ�%.2f m/s\n\n", vall);
    //
    //	printf("v֡ʱ��%.2f s\n\n", microseconds_difference);
    //	std::cout << "============================" << std::endl;
}

// ��������
void SensorReader::DecodeData(byte *c)
{
    //if (SbSumCheck(c, 0)) {
    //	Data[0] = Byte2Int(c[3], c[2]);//���ٶ�x
    //	Data[1] = Byte2Int(c[5], c[4]);//���ٶ�y
    //	Data[2] = Byte2Int(c[7], c[6]);//���ٶ�z
    //	Data[3] = Byte2Int(c[9], c[8]);//�¶� -10-11-12
    //	if (Data[3] / 100.0 <= 100) {
    //		Temperature = Data[3] / 100.0;
    //	}
    //	//# ���ٶ�
    //	a[0] = Data[0] / 32768.0 * 16;
    //	a[1] = Data[1] / 32768.0 * 16;
    //	a[2] = Data[2] / 32768.0 * 16;
    //}
    //if (SbSumCheck(c, 11)) {
    //	Data[4] = Byte2Int(c[14], c[13]);//���ٶ�x
    //	Data[5] = Byte2Int(c[16], c[15]);//���ٶ�y
    //	Data[6] = Byte2Int(c[18], c[17]);//���ٶ�z -19-20-21-22-23
    //	//# ���ٶ�
    //	w[0] = Data[4] / 32768.0 * 200;
    //	w[1] = Data[5] / 32768.0 * 200;
    //	w[2] = Data[6] / 32768.0 * 200;
    //}
    if (SbSumCheck(c, 22))
    {
        Data[8] = Byte2Int(c[25], c[24]);  //��̬��x
        Data[9] = Byte2Int(c[27], c[26]);  //��̬��y
        Data[10] = Byte2Int(c[29], c[28]); //��̬��z -30-31-32-33-34
        //# ��̬�� - rad ����
        Angle[0] = Data[8] / 32768.0 * M_PI;
        Angle[1] = Data[9] / 32768.0 * M_PI;
        Angle[2] = Data[10] / 32768.0 * M_PI;
    }
    //if (SbSumCheck(c, 33)) {
    //	Data[12] = Byte2Int(c[36], c[35]);//�ų�x
    //	Data[13] = Byte2Int(c[38], c[37]);//�ų�y
    //	Data[14] = Byte2Int(c[40], c[39]);//�ų�z
    //	//# �ų�
    //	h[0] = Data[12];
    //	h[1] = Data[13];
    //	h[2] = Data[14];
    //}
}
