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


#define TIMER_TIMEOUT (50)        // console    刷新延迟
#define GLTIMER_TIMEOUT (90)	  // 图形        刷新延迟
#define SORTIMER_TIMEOUT (4000)   // 自动SOR降噪 刷新延迟

// 主对话框类
class CCLivoxLOAMDlg : public QDialog { 
    Q_OBJECT
public:
	explicit CCLivoxLOAMDlg(ccMainAppInterface *m_appp, ccPointCloud *pointcloud, ccHObject *pcontainer, QWidget *parent = 0);
	~CCLivoxLOAMDlg();

	// console刷新 - 定时器类
	class ConsoleTimer : public QObject{
		public:
			ConsoleTimer(QTextEdit *qtextedit, QString *text);
			~ConsoleTimer();
			void handleTimeout();  //超时处理函数
			virtual void timerEvent(QTimerEvent *event);
			QTextEdit *qtextedit;
			QString *txt;
		private:
			int m_nTimerID;
	};

	// 图形刷新 - 定时器类
	class LidarGLTimer : public QObject {
		public:
			LidarGLTimer(ccMainAppInterface *mapp, ccHObject *container, ccPointCloud *pc, QTextEdit *qtedit, QString *text);
			~LidarGLTimer();
			void handleTimeout();  //超时处理函数
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

	// 点云SOR - 定时器类
	class PcSORTimer : public QObject {
	public:
		PcSORTimer(ccPointCloud *pc);
		~PcSORTimer();
		void handleTimeout();  //超时处理函数
		virtual void timerEvent(QTimerEvent *event);
		ccPointCloud *pc;
		std::mutex mtx;
	private:
		int m_nTimerID;
	};

private:
	ConsoleTimer *timer;			// console 刷新定时器
	LidarGLTimer *gltimer;			// 图形 刷新定时器
	PcSORTimer *sorTimer;			// SOR降噪 刷新定时器

	QWidget *this_parent;			// 窗口的父级指针
	ccMainAppInterface *m_app;		// CC接口指针
	ccHObject *container;			// 容器指针
	ccPointCloud *pc;				// 点云指针

	QWidget *verticalLayoutWidget;   /// * UI 元素
	QVBoxLayout *verticalLayout;	 /// *
	QHBoxLayout *horizontalLayout;	 /// *
	QLabel *label;					 /// *
	QLineEdit *BroadcastCodeInput;	 /// *
	QLabel *label_2;				 /// *
	QPushButton *StartButton;		 /// *
	QPushButton *StopButton;		 /// *
	QTextEdit *ConsoleOutput;		 /// * UI 元素

	QString ConsoleTxt;                     // console 文字
	SensorReader r;                         // IMU读取器
	LdsLidar *lidar;                        // 雷达对象
	std::thread *lidarThread;               // 雷达线程
	bool isRunning;                         // 是否正在运行的标志

	void setupUi(QDialog *Dialog);          // 初始化UI
	void retranslateUi(QDialog *Dialog);	// 翻译UI文字
	void Output2Console(QString *Txt);		// 输出到console
	void Output2Console(QString Txt);		// 输出到console
	void Scanner();							// 点云处理函数 - 启动激光雷达

	// 槽函数
public slots:
    void on_StartButton_clicked();			// 开始按钮
    void on_StopButton_clicked();			// 停止按钮
};



