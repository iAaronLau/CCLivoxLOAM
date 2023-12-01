//首先：替换所有“CCLivoxLOAM”为自己的插件名，包括info.json
//其次：打开“CCLivoxLOAM.qrc”，修改“prefix 前缀”和图标文件名为自己的插件名。然后改名为<yourPluginName>.qrc
//第三：打开info.json填写以下信息：
//	 "type" （必填）插件类型 - 必须是下面三个参数之一: "Standard", "GL", "I/O"
//	 "name" （必填）插件名称
//	 "icon" 插件图标
//	 "description" 插件描述
//	 "authors", "maintainers", "references" 作者 维护者 作者链接

#include "CCLivoxLOAMDlg.h"
#include "CCLivoxLOAM.h"
#include <QtGui>
#include <ccPointCloud.h>
#include <AsciiFilter.h>
#include <CloudSamplingTools.h>
#include <DistanceComputationTools.h>
#include <Garbage.h>
#include <GenericProgressCallback.h>
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
#include <CloudSamplingTools.h>
#include <Delaunay2dMesh.h>
#include <Jacobi.h>
#include <MeshSamplingTools.h>
#include <NormalDistribution.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>
#include <QApplication>
#include <QProgressDialog>
#include <QMainWindow>
#include <ccMesh.h>
#include <ccGLMatrixTpl.h>

using namespace CCCoreLib;

// 构造函数 - 加载插件资源
CCLivoxLOAM::CCLivoxLOAM(QObject *parent)
    : QObject(parent),
      ccStdPluginInterface(":/CC/plugin/CCLivoxLOAM/info.json"),
      m_action(nullptr) {}

// 此方法控制是否启用插件 - 取决于选中的对象类型
void CCLivoxLOAM::onNewSelection(const ccHObject::Container &selectedEntities)
{
    if (m_action == nullptr)
    {
        return;
    }

    // 只要选中了对象，就启用插件
    // m_action->setEnabled(!selectedEntities.empty());

    // 无条件启用插件
    m_action->setEnabled(true);
}

// 返回此插件所有可执行的action - CC在加载插件的时候就会执行此函数
QList<QAction *> CCLivoxLOAM::getActions()
{
    if (!m_action)
    {
        m_action = new QAction(getName(), this); // 加载插件并显示名称
        m_action->setToolTip(getDescription());  // 显示插件描述
        m_action->setIcon(getIcon());            // 显示图标

        // 绑定槽函数
        connect(m_action, &QAction::triggered, this, &CCLivoxLOAM::doAction);
    }

    return {m_action};
}

// 点击插件图标后执行的函数
void CCLivoxLOAM::doAction()
{
    if (m_app == nullptr)
    {
        Q_ASSERT(false);
        return;
    }

    // 创建容器和点云对象
    ccHObject *container;
    ccPointCloud *pc;
    container = new ccHObject("Scan001");
    pc = new ccPointCloud("RealScan");
    container->addChild(pc);
    m_app->addToDB(container, true, true);

    // 显示对话框
    CCLivoxLOAMDlg dlg(m_app, pc, container, m_app->getMainWindow());
    dlg.show();
    dlg.exec();

    // m_app->dispToConsole("[SAF] Select only one MESH!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
    // ccLog::Error(QString("Failed to load file: %1, Error # = %2").arg(txtfile).arg(res));
}