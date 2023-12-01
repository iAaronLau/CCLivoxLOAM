#pragma once

#include "SensorReader.h"
#include <string>
#include <vector>
#include <thread>
#include <iostream>
#include <chrono>
#include <ctime>
#include <mutex>
#include <PointCloud.h>
#include <ccPointCloud.h>
#include <ccRenderingTools.h>
#include <ReferenceCloud.h>
#include <ScalarFieldTools.h>
#include <ccHObjectCaster.h>
#include "CCRegistrationTools.h"

#include "livox_def.h"
#include <QtWidgets/QTextEdit>

//# ��ʱ���ƵĴ�С - ��������Ƭ��
const unsigned int miniCloudSize = 900U;

// �߳̽ڵ�ṹ�� - �̳߳صĵ�Ԫ�ṹ
typedef struct ThreadNode
{
    std::thread *t;
    ThreadNode *next;
} ThreadNode;

// ���������� - ��ʱ���ƴ���
class MiniCloudProcesser
{
public:
    // Ĭ�Ϲ��캯�� - ��ʼ����Ա����
    MiniCloudProcesser(ccPointCloud *pc, SensorReader *ar, QTextEdit *CSOutput, QString *CSTxt);
    // ��ӵ�
    void AddPoint(LivoxRawPoint *point_data);
    void AddPoint(int32_t x, int32_t y, int32_t z, int32_t r);

private:
    // ִ��ICP�任
    void ActICPTrans(ccPointCloud data_pc_full);
    // ��յ���
    void ResetPointCloud(ccPointCloud &pc);
    // �����ʱ����
    void ResetDataPc();
    // ���hello(������)
    void SayHello(int32_t x);
    // SOR���뺯��(��ʱ����)
    ccPointCloud SORFilter(ccPointCloud data_pc_full);
    // ת��ǰһ֡����
    void SavePrivousPc(ccPointCloud &from);
    // �����console
    void Output2Console(QString *Txt);
    void Output2Console(QString Txt);
    // ��ǰ���ƴ�С
    unsigned int currentPointNum;
    // ICP��׼����
    unsigned int icpCounts;

    std::thread *t;                   // �̶߳���ָ��
    std::mutex mtx;                   // �߳���
    ThreadNode tNode;                 // �̳߳� ͷ���
    unsigned int threadNum;           // �̳߳ؼ���
    QTextEdit *ConsoleOutput;         // consoleָ��
    QString *ConsoleTxt;              // console���ı�ָ��
    ccPointCloud *model_pc;           // �������ָ��
    ccPointCloud data_pc;             // ��ʱ���ƶ���
    ccPointCloud privous_pc;          // ��һ֡���ƶ���
    static SensorReader *AngelReader; // IMU��ȡ������ָ��
};