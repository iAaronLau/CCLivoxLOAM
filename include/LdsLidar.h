#pragma once

#ifndef LDS_LIDAR_H_
#define LDS_LIDAR_H_

#include <memory>
#include <vector>
#include <string>
#include "livox_def.h"
#include "livox_sdk.h"
#include "ccStdPluginInterface.h"
#include <ccPointCloud.h>
#include <QtWidgets/QTextEdit>
#include "SensorReader.h"
#include "MiniCloudProcesser.h"

// 连接状态 -  枚举
typedef enum {
  kConnectStateOff = 0,
  kConnectStateOn = 1,
  kConnectStateConfig = 2,
  kConnectStateSampling = 3,
} LidarConnectState;

// 雷达参数信息
typedef enum {
  kConfigFan = 1,
  kConfigReturnMode = 2,
  kConfigCoordinate = 4,
  kConfigImuRate = 8
} LidarConfigCodeBit;

// 相关类型
typedef enum {
  kCoordinateCartesian = 0,
  kCoordinateSpherical
} CoordinateType;

// 雷达状态数据包定义
typedef struct {
  bool enable_fan;
  uint32_t return_mode;
  uint32_t coordinate;
  uint32_t imu_rate;
  volatile uint32_t set_bits;
  volatile uint32_t get_bits;
} UserConfig;

// 雷达状态数据包定义
typedef struct {
  uint8_t handle;
  LidarConnectState connect_state;
  DeviceInfo info;
  UserConfig config;
} LidarDevice;

// 雷达数据读取器
class LdsLidar {
 public:
  // 初始化方法 - 静态获取 不要在堆栈中获取雷达读取器对象
  static LdsLidar& GetInstance() {
    static LdsLidar lds_lidar;
    return lds_lidar;
  }
  // 设置参数
  static void SetPara(ccMainAppInterface *mapp, ccPointCloud *pcloud, QTextEdit *CSOutput, QString *CSTxt, SensorReader *AgReader);
  static ccMainAppInterface *m_app;
  static ccPointCloud *pc;
  static QTextEdit *ConsoleOutput;
  static QString *ConsoleTxt;

  // 雷达是否在采集的标记
  static bool isRunning;
  // 初始化雷达
  int InitLdsLidar(std::vector<std::string>& broadcast_code_strs);
  // 反初始化雷达
  int DeInitLdsLidar(void);
  // 广播码
  std::vector<std::string> cmdline_broadcast_code;
  // 输出到Console 更新到Console
  static void Output2Console(QString *Txt);
  static void Output2Console(QString Txt);
  static void Update2Console(QString Txt);

  // 设置启动状态
  static void tStart();
  // 设置停止状态
  static void tStop();

  // IMU读取器对象指针
  static SensorReader *AngelReader;
  // 临时点云处理器对象指针
  static MiniCloudProcesser *miniCloudProcesser;

 private:
  LdsLidar();
  LdsLidar(const LdsLidar&) = delete;
  ~LdsLidar();
  LdsLidar& operator=(const LdsLidar&) = delete;

  // 带Cb的是回调函数
  static void GetLidarDataCb(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data);
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  static void StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
  static void StopSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
  static void DeviceInformationCb(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *clent_data);
  static void LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message);
  static void ControlFanCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
  static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
  static void SetCoordinateCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
  static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);

  int AddBroadcastCodeToWhitelist(const char* broadcast_code);
  void AddLocalBroadcastCode(void);
  bool FindInWhitelist(const char* broadcast_code);

  // 自动连接模式 - 启动 关闭 状态检测
  void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }

  bool auto_connect_mode_;        // 自动连接模式标记
  uint32_t whitelist_count_;      // 白名单条目数量
  volatile bool is_initialized_;  // 已初始化标记
  // 白名单
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize]; 
  // 雷达数量
  uint32_t lidar_count_;
  // 雷达实体存放处
  LidarDevice lidars_[kMaxLidarCount];
  // 数据包计数器
  uint32_t data_recveive_count_[kMaxLidarCount];
};

#endif
