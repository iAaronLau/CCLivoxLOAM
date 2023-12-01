#pragma once
#include "SensorReader.h"
#include <QDialog>
#include "ccStdPluginInterface.h"
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QtGui>
#include <QApplication>
#include <QProgressDialog>
#include <QMainWindow>
#include <string>
#include <QComboBox>
#include <QCheckBox>
#include <QProgressBar>
#include <QObject>
#include <thread>
#include "LdsLidar.h"
#include <QTimerEvent>


#define TIMER_TIMEOUT (50)        // console    ˢ���ӳ�
#define GLTIMER_TIMEOUT (90)	  // ͼ��        ˢ���ӳ�
#define SORTIMER_TIMEOUT (4000)   // �Զ�SOR���� ˢ���ӳ�

// ���Ի�����
class CCLivoxLOAMDlg : public QDialog { 
    Q_OBJECT
public:
	explicit CCLivoxLOAMDlg(ccMainAppInterface *m_appp, ccPointCloud *pointcloud, ccHObject *pcontainer, QWidget *parent = 0);
	~CCLivoxLOAMDlg();

	// consoleˢ�� - ��ʱ����
	class ConsoleTimer : public QObject{
		public:
			ConsoleTimer(QTextEdit *qtextedit, QString *text);
			~ConsoleTimer();
			void handleTimeout();  //��ʱ������
			virtual void timerEvent(QTimerEvent *event);
			QTextEdit *qtextedit;
			QString *txt;
		private:
			int m_nTimerID;
	};

	// ͼ��ˢ�� - ��ʱ����
	class LidarGLTimer : public QObject {
		public:
			LidarGLTimer(ccMainAppInterface *mapp, ccHObject *container, ccPointCloud *pc, QTextEdit *qtedit, QString *text);
			~LidarGLTimer();
			void handleTimeout();  //��ʱ������
			virtual void timerEvent(QTimerEvent *event);
			ccMainAppInterface *m_app;
			ccHObject *container;
			ccPointCloud *pc;
			unsigned int refreshCount;
			QTextEdit *qtextedit;
			QString *txt;
		private:
			int m_nTimerID;
	};

	// ����SOR - ��ʱ����
	class PcSORTimer : public QObject {
	public:
		PcSORTimer(ccPointCloud *pc);
		~PcSORTimer();
		void handleTimeout();  //��ʱ������
		virtual void timerEvent(QTimerEvent *event);
		ccPointCloud *pc;
		std::mutex mtx;
	private:
		int m_nTimerID;
	};

private:
	ConsoleTimer *timer;			// console ˢ�¶�ʱ��
	LidarGLTimer *gltimer;			// ͼ�� ˢ�¶�ʱ��
	PcSORTimer *sorTimer;			// SOR���� ˢ�¶�ʱ��

	QWidget *this_parent;			// ���ڵĸ���ָ��
	ccMainAppInterface *m_app;		// CC�ӿ�ָ��
	ccHObject *container;			// ����ָ��
	ccPointCloud *pc;				// ����ָ��

	QWidget *verticalLayoutWidget;   /// * UI Ԫ��
	QVBoxLayout *verticalLayout;	 /// *
	QHBoxLayout *horizontalLayout;	 /// *
	QLabel *label;					 /// *
	QLineEdit *BroadcastCodeInput;	 /// *
	QLabel *label_2;				 /// *
	QPushButton *StartButton;		 /// *
	QPushButton *StopButton;		 /// *
	QTextEdit *ConsoleOutput;		 /// * UI Ԫ��

	QString ConsoleTxt;                     // console ����
	SensorReader r;                         // IMU��ȡ��
	LdsLidar *lidar;                        // �״����
	std::thread *lidarThread;               // �״��߳�
	bool isRunning;                         // �Ƿ��������еı�־

	void setupUi(QDialog *Dialog);          // ��ʼ��UI
	void retranslateUi(QDialog *Dialog);	// ����UI����
	void Output2Console(QString *Txt);		// �����console
	void Output2Console(QString Txt);		// �����console
	void Scanner();							// ���ƴ����� - ���������״�

	// �ۺ���
public slots:
    void on_StartButton_clicked();			// ��ʼ��ť
    void on_StopButton_clicked();			// ֹͣ��ť
};



