#pragma once
#include "CCLivoxLOAMDlg.h"
#include "ccStdPluginInterface.h"


// ����ӿ� - �̳� QObject �� CC��׼����ӿ�
class CCLivoxLOAM : public QObject, public ccStdPluginInterface{
    Q_OBJECT
    Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
	// �����ID �������ļ������·��
    Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.CCLivoxLOAM" FILE "../info.json")

public:
    explicit CCLivoxLOAM(QObject *parent = nullptr);
    ~CCLivoxLOAM() override = default;
    void onNewSelection(const ccHObject::Container &selectedEntities) override;
    QList<QAction *> getActions() override;
    void doAction(); // ���������ִ�еĺ���

private:
    QAction *m_action;
};
