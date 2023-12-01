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

// ����״̬ -  ö��
typedef enum {
  kConnectStateOff = 0,
  kConnectStateOn = 1,
  kConnectStateConfig = 2,
  kConnectStateSampling = 3,
} LidarConnectState;

// �״������Ϣ
typedef enum {
  kConfigFan = 1,
  kConfigReturnMode = 2,
  kConfigCoordinate = 4,
  kConfigImuRate = 8
} LidarConfigCodeBit;

// �������
typedef enum {
  kCoordinateCartesian = 0,
  kCoordinateSpherical
} CoordinateType;

// �״�״̬���ݰ�����
typedef struct {
  bool enable_fan;
  uint32_t return_mode;
  uint32_t coordinate;
  uint32_t imu_rate;
  volatile uint32_t set_bits;
  volatile uint32_t get_bits;
} UserConfig;

// �״�״̬���ݰ�����
typedef struct {
  uint8_t handle;
  LidarConnectState connect_state;
  DeviceInfo info;
  UserConfig config;
} LidarDevice;

// �״����ݶ�ȡ��
class LdsLidar {
 public:
  // ��ʼ������ - ��̬��ȡ ��Ҫ�ڶ�ջ�л�ȡ�״��ȡ������
  static LdsLidar& GetInstance() {
    static LdsLidar lds_lidar;
    return lds_lidar;
  }
  // ���ò���
  static void SetPara(ccMainAppInterface *mapp, ccPointCloud *pcloud, QTextEdit *CSOutput, QString *CSTxt, SensorReader *AgReader);
  static ccMainAppInterface *m_app;
  static ccPointCloud *pc;
  static QTextEdit *ConsoleOutput;
  static QString *ConsoleTxt;

  // �״��Ƿ��ڲɼ��ı��
  static bool isRunning;
  // ��ʼ���״�
  int InitLdsLidar(std::vector<std::string>& broadcast_code_strs);
  // ����ʼ���״�
  int DeInitLdsLidar(void);
  // �㲥��
  std::vector<std::string> cmdline_broadcast_code;
  // �����Console ���µ�Console
  static void Output2Console(QString *Txt);
  static void Output2Console(QString Txt);
  static void Update2Console(QString Txt);

  // ��������״̬
  static void tStart();
  // ����ֹͣ״̬
  static void tStop();

  // IMU��ȡ������ָ��
  static SensorReader *AngelReader;
  // ��ʱ���ƴ���������ָ��
  static MiniCloudProcesser *miniCloudProcesser;

 private:
  LdsLidar();
  LdsLidar(const LdsLidar&) = delete;
  ~LdsLidar();
  LdsLidar& operator=(const LdsLidar&) = delete;

  // ��Cb���ǻص�����
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

  // �Զ�����ģʽ - ���� �ر� ״̬���
  void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }

  bool auto_connect_mode_;        // �Զ�����ģʽ���
  uint32_t whitelist_count_;      // ��������Ŀ����
  volatile bool is_initialized_;  // �ѳ�ʼ�����
  // ������
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize]; 
  // �״�����
  uint32_t lidar_count_;
  // �״�ʵ���Ŵ�
  LidarDevice lidars_[kMaxLidarCount];
  // ���ݰ�������
  uint32_t data_recveive_count_[kMaxLidarCount];
};

#endif
