#include "MiniCloudProcesser.h"
#include "LdsLidar.h"
#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>
#include <GeometricalAnalysisTools.h>
#include <Jacobi.h>
#include <KdTree.h>
#include <ManualSegmentationTools.h>
#include <NormalDistribution.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>
#include <ScalarFieldTools.h>
#include <ccHObjectCaster.h>
#include <cmath>

// �㲥������
static const char *local_broadcast_code_list[] = {
    "0TFDH9U0065HTF1",
};

// ��ʼ����̬���� �����ⲿ��ָ��
ccMainAppInterface *LdsLidar::m_app = nullptr;
ccPointCloud *LdsLidar::pc = nullptr;
QTextEdit *LdsLidar::ConsoleOutput = nullptr;
QString *LdsLidar::ConsoleTxt = nullptr;
SensorReader *LdsLidar::AngelReader = nullptr;
MiniCloudProcesser *LdsLidar::miniCloudProcesser = nullptr;
bool LdsLidar::isRunning = false;

LdsLidar *g_lidars = nullptr;

// Ĭ�Ϲ��캯�� ��ʼ��״̬����
LdsLidar::LdsLidar()
{
    auto_connect_mode_ = true;
    whitelist_count_ = 0;
    is_initialized_ = false;
    lidar_count_ = 0;
    memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
    memset(lidars_, 0, sizeof(lidars_));
    for (uint32_t i = 0; i < kMaxLidarCount; i++)
    {
        lidars_[i].handle = kMaxLidarCount;
        lidars_[i].connect_state = kConnectStateOff;
    }
}

LdsLidar::~LdsLidar()
{
}

void LdsLidar::SetPara(ccMainAppInterface *mapp, ccPointCloud *pcloud, QTextEdit *CSOutput, QString *CSTxt, SensorReader *AgReader)
{
    isRunning = false;
    m_app = mapp;
    pc = pcloud;
    ConsoleOutput = CSOutput;
    ConsoleTxt = CSTxt;
    AngelReader = AgReader;
    miniCloudProcesser = new MiniCloudProcesser(pc, AngelReader, ConsoleOutput, ConsoleTxt);
}

// ��ʼ���״�
int LdsLidar::InitLdsLidar(std::vector<std::string> &broadcast_code_strs)
{

    // �����Ѿ���ʼ�����״�
    if (is_initialized_)
    {
        Output2Console("LiDAR data source is Already Inited");
        return -1;
    }

    //���Գ�ʼ��
    if (!Init())
    { //���û�ܳ�ʼ�����򷴳�ʼ��
        Uninit();
        Output2Console("Livox-SDK Init Fail\n");
        return -1;
    }

    // ��ȡSDK�汾
    LivoxSdkVersion _sdkversion;
    GetLivoxSdkVersion(&_sdkversion);
    Output2Console(QString("Livox SDK version %1.%2.%3").arg(_sdkversion.major).arg(_sdkversion.minor).arg(_sdkversion.patch));

    SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
    SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

    // ��ӹ㲥�뵽������ - ����CMD
    for (auto input_str : broadcast_code_strs)
    {
        LdsLidar::AddBroadcastCodeToWhitelist(input_str.c_str());
    }
    // ��ӹ㲥�뵽������ - ���Ա��ر���
    LdsLidar::AddLocalBroadcastCode();

    // �������������,��ʹ�ð�����,�ر��Զ�����ģʽ. ����ʹ���Զ�����ģʽ.
    if (whitelist_count_)
    {
        LdsLidar::DisableAutoConnectMode();
        Output2Console("Disable Auto Connect Mode!");
        Output2Console("List all broadcast code in whiltelist:");
        for (uint32_t i = 0; i < whitelist_count_; i++)
        {
            Output2Console(QString("# %1").arg(broadcast_code_whitelist_[i]));
        }
    }
    else
    {
        LdsLidar::EnableAutoConnectMode();
        Output2Console("No broadcast code was added to whitelist, swith to automatic connection mode!");
    }

    /** �����״����������� */
    if (!Start())
    {
        Uninit();
        Output2Console("Start Livox-SDK Init FAIL");
        return -1;
    }

    // ��ȫ�ֱ��������浱ǰ�����ַ
    if (g_lidars == nullptr)
    {
        g_lidars = this;
    }
    // ��ʼ�����
    is_initialized_ = true;
    Output2Console("Livox-SDK init SUCCESS");
    return 0;
}

// ����ʼ��
int LdsLidar::DeInitLdsLidar(void)
{
    //���統ǰû�г�ʼ��, ���÷���ʼ��
    if (!is_initialized_)
    {
        Output2Console("LiDAR data source is not exits");
        return -1;
    }
    // ����ʼ��
    Uninit();
    Output2Console("Livox SDK Deinit Completely");
    return 0;
}

/** ���´�Cb�ĺ������Ǵ����״����ݵĻص�����, �����Ǿ�̬����, ��Ҫ���ع�ע GetLidarDataCb() -------*/

/** �������� */
void LdsLidar::GetLidarDataCb(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)
{
    using namespace std;
    // �ھ�̬�����ڴ�������״����ݶ�ȡ����ָ��
    LdsLidar *lidar_this = static_cast<LdsLidar *>(client_data);
    // data = �״�����
    LivoxEthPacket *eth_packet = data;
    if (!data || !data_num || (handle >= kMaxLidarCount))
    {
        return;
    }
    // ��������ݰ���״̬��������,�������ӵ�Ĳ���
    if (eth_packet && isRunning)
    {
        // ���ݰ�����
        lidar_this->data_recveive_count_[handle]++;
        // ���ݰ�����ת��
        LivoxRawPoint *ppoint_data = (LivoxRawPoint *)data->data;

        // ��ӵ�
        if (miniCloudProcesser)
        {
            miniCloudProcesser->AddPoint(ppoint_data->x, ppoint_data->y, ppoint_data->z, ppoint_data->reflectivity);
        }
        // ���ݰ�����, ��console���
        /*if (lidar_this->data_recveive_count_[handle] % 100 == 0) {
	  Update2Console(QString("### receive packet count %1 %2").arg(handle).arg(lidar_this->data_recveive_count_[handle]));
    }*/
    }
}

// �豸�㲥�ص�����
void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info)
{
    if (info == nullptr)
    {
        return;
    }

    if (info->dev_type == kDeviceTypeHub)
    {
        printf("In lidar mode, couldn't connect a hub : %s\n", info->broadcast_code);
        return;
    }

    if (g_lidars->IsAutoConnectMode())
    {
        printf("In automatic connection mode, will connect %s\n", info->broadcast_code);
    }
    else
    {
        if (!g_lidars->FindInWhitelist(info->broadcast_code))
        {
            printf("Not in the whitelist, please add %s to if want to connect!\n",
                   info->broadcast_code);
            return;
        }
    }

    livox_status result = kStatusFailure;
    uint8_t handle = 0;
    result = AddLidarToConnect(info->broadcast_code, &handle);
    if (result == kStatusSuccess && handle < kMaxLidarCount)
    {
        SetDataCallback(handle, LdsLidar::GetLidarDataCb, (void *)g_lidars);
        LidarDevice *p_lidar = &(g_lidars->lidars_[handle]);
        p_lidar->handle = handle;
        p_lidar->connect_state = kConnectStateOff;
        p_lidar->config.enable_fan = true;
        p_lidar->config.return_mode = kStrongestReturn;
        p_lidar->config.coordinate = kCoordinateCartesian;
        p_lidar->config.imu_rate = kImuFreq200Hz;
    }
    else
    {
        printf("Add lidar to connect is failed : %d %d \n", result, handle);
    }
}

/** �豸״̬�ı�Ļص����� */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type)
{
    if (info == nullptr)
    {
        return;
    }

    uint8_t handle = info->handle;
    if (handle >= kMaxLidarCount)
    {
        return;
    }

    LidarDevice *p_lidar = &(g_lidars->lidars_[handle]);
    if (type == kEventConnect)
    {
        QueryDeviceInformation(handle, DeviceInformationCb, g_lidars);
        if (p_lidar->connect_state == kConnectStateOff)
        {
            p_lidar->connect_state = kConnectStateOn;
            p_lidar->info = *info;
        }
        printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
    }
    else if (type == kEventDisconnect)
    {
        p_lidar->connect_state = kConnectStateOff;
        printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
    }
    else if (type == kEventStateChange)
    {
        p_lidar->info = *info;
        printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
    }

    if (p_lidar->connect_state == kConnectStateOn)
    {
        printf("Device Working State %d\n", p_lidar->info.state);
        if (p_lidar->info.state == kLidarStateInit)
        {
            printf("Device State Change Progress %u\n", p_lidar->info.status.progress);
        }
        else
        {
            printf("Device State Error Code 0X%08x\n", p_lidar->info.status.status_code.error_code);
        }
        printf("Device feature %d\n", p_lidar->info.feature);
        SetErrorMessageCallback(handle, LdsLidar::LidarErrorStatusCb);

        /** Config lidar parameter */
        if (p_lidar->info.state == kLidarStateNormal)
        {
            if (p_lidar->config.coordinate != 0)
            {
                SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
            }
            else
            {
                SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
            }
            p_lidar->config.set_bits |= kConfigCoordinate;

            if (kDeviceTypeLidarMid40 != info->type)
            {
                LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
                                             LdsLidar::SetPointCloudReturnModeCb, g_lidars);
                p_lidar->config.set_bits |= kConfigReturnMode;
            }

            if (kDeviceTypeLidarMid40 != info->type && kDeviceTypeLidarMid70 != info->type)
            {
                LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                                         LdsLidar::SetImuRatePushFrequencyCb, g_lidars);
                p_lidar->config.set_bits |= kConfigImuRate;
            }

            p_lidar->connect_state = kConnectStateConfig;
        }
    }
}

/** ��ѯӲ���汾 */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *client_data)
{
    if (status != kStatusSuccess)
    {
        printf("Device Query Informations Failed : %d\n", status);
    }
    if (ack)
    {
        printf("firm ver: %d.%d.%d.%d\n",
               ack->firmware_version[0],
               ack->firmware_version[1],
               ack->firmware_version[2],
               ack->firmware_version[3]);
    }
}

/** ������Ϣ�ص����� */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message)
{
    static uint32_t error_message_count = 0;
    if (message != NULL)
    {
        ++error_message_count;
        if (0 == (error_message_count % 100))
        {
            printf("handle: %u\n", handle);
            printf("temp_status : %u\n", message->lidar_error_code.temp_status);
            printf("volt_status : %u\n", message->lidar_error_code.volt_status);
            printf("motor_status : %u\n", message->lidar_error_code.motor_status);
            printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
            printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
            printf("pps_status : %u\n", message->lidar_error_code.device_status);
            printf("fan_status : %u\n", message->lidar_error_code.fan_status);
            printf("self_heating : %u\n", message->lidar_error_code.self_heating);
            printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
            printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
            printf("system_status : %u\n", message->lidar_error_code.system_status);
        }
    }
}

/** ���ȿ��� */
void LdsLidar::ControlFanCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (handle >= kMaxLidarCount)
    {
        return;
    }
    LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

    if (status == kStatusSuccess)
    {
        p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
        printf("Set return mode success!\n");

        if (!p_lidar->config.set_bits)
        {
            LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
            p_lidar->connect_state = kConnectStateSampling;
        }
    }
    else
    {
        LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
                                     LdsLidar::SetPointCloudReturnModeCb, lds_lidar);
        printf("Set return mode fail, try again!\n");
    }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (handle >= kMaxLidarCount)
    {
        return;
    }
    LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

    if (status == kStatusSuccess)
    {
        p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
        printf("Set coordinate success!\n");

        if (!p_lidar->config.set_bits)
        {
            LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
            p_lidar->connect_state = kConnectStateSampling;
        }
    }
    else
    {
        if (p_lidar->config.coordinate != 0)
        {
            SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
        }
        else
        {
            SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
        }

        printf("Set coordinate fail, try again!\n");
    }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (handle >= kMaxLidarCount)
    {
        return;
    }
    LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

    if (status == kStatusSuccess)
    {
        p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
        printf("Set imu rate success!\n");

        if (!p_lidar->config.set_bits)
        {
            LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
            p_lidar->connect_state = kConnectStateSampling;
        }
    }
    else
    {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                                 LdsLidar::SetImuRatePushFrequencyCb, lds_lidar);
        printf("Set imu rate fail, try again!\n");
    }
}

/** ��ʼ�����Ļص����� */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (handle >= kMaxLidarCount)
    {
        return;
    }

    LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
    if (status == kStatusSuccess)
    {
        if (response != 0)
        {
            p_lidar->connect_state = kConnectStateOn;
            printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n",
                   status, handle, response);
        }
        else
        {
            printf("Lidar start sample success\n");
        }
    }
    else if (status == kStatusTimeout)
    {
        p_lidar->connect_state = kConnectStateOn;
        printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n",
               status, handle, response);
    }
}

/** ֹͣ�����ص����� */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
}

/** ��ӹ㲥������� */
int LdsLidar::AddBroadcastCodeToWhitelist(const char *bd_code)
{
    if (!bd_code || (strlen(bd_code) > kBroadcastCodeSize) || (whitelist_count_ >= kMaxLidarCount))
    {
        return -1;
    }

    if (LdsLidar::FindInWhitelist(bd_code))
    {
        printf("%s is alrealy exist!\n", bd_code);
        return -1;
    }

    strcpy(broadcast_code_whitelist_[whitelist_count_], bd_code);
    ++whitelist_count_;

    return 0;
}

/** ��ӱ��ع㲥�� */
void LdsLidar::AddLocalBroadcastCode(void)
{
    for (size_t i = 0; i < sizeof(local_broadcast_code_list) / sizeof(intptr_t); ++i)
    {
        std::string invalid_bd = "000000000";
        printf("Local broadcast code : %s\n", local_broadcast_code_list[i]);
        if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) + 1) &&
            (nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str())))
        {
            LdsLidar::AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
        }
        else
        {
            printf("Invalid local broadcast code : %s\n", local_broadcast_code_list[i]);
        }
    }
}

bool LdsLidar::FindInWhitelist(const char *bd_code)
{
    if (!bd_code)
    {
        return false;
    }

    for (uint32_t i = 0; i < whitelist_count_; i++)
    {
        if (strncmp(bd_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0)
        {
            return true;
        }
    }

    return false;
}

// �����console
void LdsLidar::Output2Console(QString *Txt)
{
    ConsoleTxt->append("\n");
    ConsoleTxt->append(*Txt);
}

void LdsLidar::Output2Console(QString Txt)
{
    Output2Console(&Txt);
}

void LdsLidar::Update2Console(QString Txt)
{
    if (ConsoleTxt->lastIndexOf("### receive") >= 0)
    {
        ConsoleTxt->chop(ConsoleTxt->length() - ConsoleTxt->lastIndexOf("### receive") + 1);
    }
    Output2Console(&Txt);
}

// ����������ֹͣ��״̬
void LdsLidar::tStart()
{
    isRunning = true;
    Output2Console("Started");
}
void LdsLidar::tStop()
{
    isRunning = false;
    Output2Console("Stopped");
}