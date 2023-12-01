#pragma once
#include "CCLivoxLOAMDlg.h"
#include "ccStdPluginInterface.h"


// 插件接口 - 继承 QObject 和 CC标准插件接口
class CCLivoxLOAM : public QObject, public ccStdPluginInterface{
    Q_OBJECT
    Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
	// 插件的ID 和配置文件的相对路径
    Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.CCLivoxLOAM" FILE "../info.json")

public:
    explicit CCLivoxLOAM(QObject *parent = nullptr);
    ~CCLivoxLOAM() override = default;
    void onNewSelection(const ccHObject::Container &selectedEntities) override;
    QList<QAction *> getActions() override;
    void doAction(); // 触发插件后执行的函数

private:
    QAction *m_action;
};
