#include "CCLivoxLOAMDlg.h"
#include <QFileDialog>
#include <iostream>
#include <QtGui>
#include <QApplication>
#include <QProgressDialog>
#include <QMainWindow>
#include <ccPointCloud.h>
#include <exception>
#include <BinFilter.h>
#include <QComboBox>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QString>
#include <QStringList>
#include <QByteArray>
#include <QMap>
#include <QVector>
#include <string>
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
#include <QProgressDialog>
#include <QRegularExpression>
#include <QProgressBar>
#include <ccGLWidget.h>
#include <ccRenderingTools.h>

using namespace CCCoreLib;

static std::vector<std::string> cmdline_broadcast_code; // �����״�㲥��
static bool canRefresh = true;                          // �Ƿ����ˢ��GL�ı�־

CCLivoxLOAMDlg::CCLivoxLOAMDlg(ccMainAppInterface *m_appp, 
								ccPointCloud *pointcloud, 
								ccHObject *pcontainer, 
								QWidget *parent) 
								: QDialog(parent){
	// ��ʼ����Ա����
    this_parent = parent;
    m_app = m_appp;
    setupUi(this);
	isRunning = false;
	pc = pointcloud;
	container = pcontainer;
	container->addChild(pc);
	m_app->addToDB(container, true, true);
	static_cast<ccGLWindow*>(container->getDisplay())->setVisible(true);

	// ��ʼ�����Ʋ���
	pc->setDisplay(m_app->getActiveGLWindow());
	pc->addScalarField("DefaultScalarField");
	pc->enableScalarField();
	pc->showSF(true);
	pc->setCurrentDisplayedScalarField(0);

	// ��ʼ��ָ��
	lidar = nullptr;
	lidarThread = nullptr;
	timer = nullptr;
	gltimer = nullptr;
	sorTimer = nullptr;
}

void CCLivoxLOAMDlg::Scanner() {
	m_app->dispToConsole(QString("New Thread"), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	if (!lidar) {
		return;
	}
	// ��ʼ���״����㲥��
	int ret = lidar->InitLdsLidar(cmdline_broadcast_code);
	if (!ret) { // ��ʼ���ɹ�
		isRunning = true;
		Output2Console("Init Lidar SUCCESS!");
	}
	else {      // ��ʼ��ʧ��
		isRunning = false;
		// ���ð�ť״̬
		StopButton->setDisabled(TRUE);
		StartButton->setEnabled(TRUE);
		BroadcastCodeInput->setEnabled(TRUE);
		// ��� console ��Ϣ 
		Output2Console("Stoped.");
		Output2Console("<<<<<<<<<<<<<<<<<<<<<<<");
	}

	// �����̲߳��˳�
	while (isRunning == true) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// �߳��˳�ǰ���еĲ���
	lidar->DeInitLdsLidar();
	isRunning = false;
	// ���ð�ť״̬
	StopButton->setDisabled(TRUE);
	StartButton->setEnabled(TRUE);
	BroadcastCodeInput->setEnabled(TRUE);
	m_app->dispToConsole(QString("Thread Quit"), ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

void CCLivoxLOAMDlg::on_StartButton_clicked() {
	Output2Console((">>>>>>>>>>>>>>>>>>>>>>"));
	// ���ð�ť״̬
	StopButton->setEnabled(TRUE);
	StartButton->setDisabled(TRUE);
	BroadcastCodeInput->setDisabled(TRUE);

	// SOR���붨ʱ��
	if (sorTimer == nullptr) {
		sorTimer = new PcSORTimer(pc);
	}
	// console��ˢ�¶�ʱ��
	if (timer == nullptr) {
		timer = new ConsoleTimer(ConsoleOutput, &ConsoleTxt);
	}
	// ͼ�ε�ˢ�¶�ʱ��
	if (gltimer == nullptr) {
		gltimer = new LidarGLTimer(m_app, container, pc, ConsoleOutput, &ConsoleTxt);
	}
	// ����IMU��ȡ��
	r.Start();
	// �����״��߳�
	if (this->lidar == nullptr) {
		//cmdline_broadcast_code.push_back(BroadcastCodeInput->text().toStdString());
		cmdline_broadcast_code.push_back("0TFDH9U0065HTF1");
		lidar = &LdsLidar::GetInstance();
		// �����״��Ҫ�Ĳ��� - ����ָ�� console IMU��ȡ��
		lidar->SetPara(m_app, pc, ConsoleOutput, &ConsoleTxt, &r); 
		// �����״�״̬Ϊ����
		lidar->tStart();
		// �����״��߳�
		this->lidarThread = new std::thread(&CCLivoxLOAMDlg::Scanner, this);
	}
	else {
		lidar->tStart();
	}
}

void CCLivoxLOAMDlg::on_StopButton_clicked() {
	// �����״�״̬Ϊֹͣ
	if (this->lidar == nullptr) {
		lidar->tStop();
	}
	// ��ֹ���ж�ʱ��
	if (this->sorTimer != nullptr && this->sorTimer != NULL) {
		delete sorTimer;
		sorTimer = nullptr;
	}
	if (this->timer != nullptr && this->timer != NULL) {
		delete timer;
		timer = nullptr;
	}
	if (this->gltimer != nullptr && this->gltimer != NULL) {
		delete gltimer;
		gltimer = nullptr;
	}

	Output2Console("<<<<<<<<<<<<<<<<<<<<<<<");
	// ���ð�ť״̬
	StopButton->setDisabled(TRUE);
	StartButton->setEnabled(TRUE);
	BroadcastCodeInput->setEnabled(TRUE);
}

void CCLivoxLOAMDlg::Output2Console(QString *Txt) {
	ConsoleTxt.append("\n");
	ConsoleTxt.append(*Txt);
}

void CCLivoxLOAMDlg::Output2Console(QString Txt) {
	Output2Console(&Txt);
}

CCLivoxLOAMDlg::~CCLivoxLOAMDlg() {
	isRunning = false;
	// �ȴ��״��߳��˳�
	if (lidarThread && lidarThread->joinable()) {
		lidarThread->join();
	}
	// �����״�״̬Ϊֹͣ ��ֹ���ж�ʱ��
	on_StopButton_clicked();
}

void CCLivoxLOAMDlg::setupUi(QDialog *Dialog) {
	if (Dialog->objectName().isEmpty())
		Dialog->setObjectName(QString::fromUtf8("CCLivoxLOAM"));
	Dialog->resize(532, 331);
	verticalLayoutWidget = new QWidget(Dialog);
	verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
	verticalLayoutWidget->setGeometry(QRect(10, 10, 511, 311));
	verticalLayout = new QVBoxLayout(verticalLayoutWidget);
	verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
	verticalLayout->setContentsMargins(0, 0, 0, 0);
	horizontalLayout = new QHBoxLayout();
	horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
	label = new QLabel(verticalLayoutWidget);
	label->setObjectName(QString::fromUtf8("label"));
	horizontalLayout->addWidget(label);
	BroadcastCodeInput = new QLineEdit(verticalLayoutWidget);
	BroadcastCodeInput->setObjectName(QString::fromUtf8("BroadcastCodeInput"));
	horizontalLayout->addWidget(BroadcastCodeInput);
	label_2 = new QLabel(verticalLayoutWidget);
	label_2->setObjectName(QString::fromUtf8("label_2"));
	horizontalLayout->addWidget(label_2);
	StartButton = new QPushButton(verticalLayoutWidget);
	StartButton->setObjectName(QString::fromUtf8("StartButton"));
	horizontalLayout->addWidget(StartButton);
	StopButton = new QPushButton(verticalLayoutWidget);
	StopButton->setObjectName(QString::fromUtf8("StopButton"));
	horizontalLayout->addWidget(StopButton);
	verticalLayout->addLayout(horizontalLayout);
	ConsoleOutput = new QTextEdit(verticalLayoutWidget);
	ConsoleOutput->setObjectName(QString::fromUtf8("ConsoleOutput"));
	verticalLayout->addWidget(ConsoleOutput);
	retranslateUi(Dialog);
	QMetaObject::connectSlotsByName(Dialog);

	ConsoleTxt = "Ready.";
	ConsoleOutput->setPlainText(ConsoleTxt);
	ConsoleOutput->setEnabled(TRUE);
	ConsoleOutput->setVisible(TRUE);
	StopButton->setDisabled(TRUE);
}

void CCLivoxLOAMDlg::retranslateUi(QDialog *CCLivoxLOAM){
	CCLivoxLOAM->setWindowTitle(QApplication::translate("CCLivoxLOAM", "CCLivoxLOAM", nullptr));
	label->setText(QApplication::translate("CCLivoxLOAM", "\345\271\277\346\222\255\347\240\201\357\274\232", nullptr));
	BroadcastCodeInput->setText(QApplication::translate("CCLivoxLOAM", "0TFDH9U0065HTF1", nullptr));
	label_2->setText(QApplication::translate("CCLivoxLOAM", "(\350\207\252\345\212\250\346\243\200\346\265\213\345\210\231\347\275\256\347\251\272)", nullptr));
	StartButton->setText(QApplication::translate("CCLivoxLOAM", "\345\220\257\345\212\250", nullptr));
	StopButton->setText(QApplication::translate("CCLivoxLOAM", "\345\201\234\346\255\242", nullptr));
} 

CCLivoxLOAMDlg::ConsoleTimer::ConsoleTimer(QTextEdit *qtedit, QString *text) {
	m_nTimerID = this->startTimer(TIMER_TIMEOUT);
	qtextedit = qtedit;
	txt = text;
}

CCLivoxLOAMDlg::ConsoleTimer::~ConsoleTimer(){
	killTimer(m_nTimerID);
}

void CCLivoxLOAMDlg::ConsoleTimer::timerEvent(QTimerEvent *event){
	if (event->timerId() == m_nTimerID) {
		handleTimeout();
	}
}

void CCLivoxLOAMDlg::ConsoleTimer::handleTimeout(){
	qtextedit->setPlainText(*txt);
	qtextedit->moveCursor(QTextCursor::End);
}

CCLivoxLOAMDlg::LidarGLTimer::LidarGLTimer(ccMainAppInterface *mapp, ccHObject *cchobj, ccPointCloud *ccpc, QTextEdit *qtedit, QString *text) {
	m_nTimerID = this->startTimer(GLTIMER_TIMEOUT);
	m_app = mapp;
	container = cchobj;
	pc = ccpc;
	refreshCount = 0;
	qtextedit = qtedit;
	txt = text;
}

CCLivoxLOAMDlg::LidarGLTimer::~LidarGLTimer() {
	killTimer(m_nTimerID);
}

void CCLivoxLOAMDlg::LidarGLTimer::timerEvent(QTimerEvent *event) {
	if (event->timerId() == m_nTimerID) {
		handleTimeout();
	}
}

void CCLivoxLOAMDlg::LidarGLTimer::handleTimeout() {
	refreshCount++; // ��¼ˢ�´���
	if (canRefresh) {
		// ˢ��GLͼ��
		pc->redrawDisplay();
		// �ض�����������������
		if (refreshCount==10 || refreshCount == 20 || refreshCount == 50) {
			m_app->removeFromDB(container,false);
			m_app->addToDB(container);
		}
	}
}

CCLivoxLOAMDlg::PcSORTimer::PcSORTimer(ccPointCloud *ccpc) {
	m_nTimerID = this->startTimer(SORTIMER_TIMEOUT);
	pc = ccpc;
}

CCLivoxLOAMDlg::PcSORTimer::~PcSORTimer() {
	killTimer(m_nTimerID);
}

void CCLivoxLOAMDlg::PcSORTimer::timerEvent(QTimerEvent *event) {
	if (event->timerId() == m_nTimerID) {
		//handleTimeout();
	}
}

void CCLivoxLOAMDlg::PcSORTimer::handleTimeout() {
	if (pc->size()>=10) {
		// SOR�˲�(����)
		CCCoreLib::ReferenceCloud* selection = 
			CCCoreLib::CloudSamplingTools::sorFilter(pc, 3, 1.0, 0, 0);
		if (selection) {
			canRefresh = false;
			if (selection->size() != pc->size()) {
				ccPointCloud* cleanCloud = pc->partialClone(selection);
				//delete pc; pc = nullptr;
				pc = cleanCloud;
				//cleanCloud->clone(pc);
				ccLog::LogMessage("[SOR] Filter Performed.",0);
				/*if (cleanCloud) {
					mtx.lock();
					pc->clear();
					pc->clearFWFData();
					pc->clearLOD();
					pc->deleteAllScalarFields();
					pc->deleteOctree();
					pc->removeAllChildren();
					pc->removeAllClipPlanes();
					pc->removeGrids();
					pc->addScalarField("DefaultScalarField");
					pc->enableScalarField();
					pc->showSF(true);
					pc->setCurrentDisplayedScalarField(0);
					for (unsigned int i = 0; i < cleanCloud->size(); i++) {
						pc->reserve(1);
						pc->addPoint(CCVector3(cleanCloud->getPoint(i)->x, cleanCloud->getPoint(i)->y, cleanCloud->getPoint(i)->z));
						pc->getScalarField(0)->addElement(cleanCloud->getScalarField(0)->getValue(i));
					}
					mtx.unlock();
				}*/
			}
			canRefresh = true;
		}
		else {
			ccLog::LogMessage("[SOR] Filter NNN.",0);
		}
	}
	else {
		ccLog::LogMessage("[SOR] Filter NNN.",0);
	}
}