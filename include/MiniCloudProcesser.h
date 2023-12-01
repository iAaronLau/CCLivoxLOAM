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

//# 临时点云的大小 - 点云增量片段
const unsigned int miniCloudSize = 900U;

// 线程节点结构体 - 线程池的单元结构
typedef struct ThreadNode
{
    std::thread *t;
    ThreadNode *next;
} ThreadNode;

// 增量处理器 - 临时点云处理
class MiniCloudProcesser
{
public:
    // 默认构造函数 - 初始化成员变量
    MiniCloudProcesser(ccPointCloud *pc, SensorReader *ar, QTextEdit *CSOutput, QString *CSTxt);
    // 添加点
    void AddPoint(LivoxRawPoint *point_data);
    void AddPoint(int32_t x, int32_t y, int32_t z, int32_t r);

private:
    // 执行ICP变换
    void ActICPTrans(ccPointCloud data_pc_full);
    // 清空点云
    void ResetPointCloud(ccPointCloud &pc);
    // 清空临时点云
    void ResetDataPc();
    // 输出hello(已弃用)
    void SayHello(int32_t x);
    // SOR降噪函数(暂时弃用)
    ccPointCloud SORFilter(ccPointCloud data_pc_full);
    // 转存前一帧点云
    void SavePrivousPc(ccPointCloud &from);
    // 输出到console
    void Output2Console(QString *Txt);
    void Output2Console(QString Txt);
    // 当前点云大小
    unsigned int currentPointNum;
    // ICP配准计数
    unsigned int icpCounts;

    std::thread *t;                   // 线程对象指针
    std::mutex mtx;                   // 线程锁
    ThreadNode tNode;                 // 线程池 头结点
    unsigned int threadNum;           // 线程池计数
    QTextEdit *ConsoleOutput;         // console指针
    QString *ConsoleTxt;              // console的文本指针
    ccPointCloud *model_pc;           // 主体点云指针
    ccPointCloud data_pc;             // 临时点云对象
    ccPointCloud privous_pc;          // 上一帧点云对象
    static SensorReader *AngelReader; // IMU读取器对象指针
};