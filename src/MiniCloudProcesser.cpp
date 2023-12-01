#include "MiniCloudProcesser.h"
#include "CCRegistrationTools.h"
#include <CloudSamplingTools.h>

// 初始化静态变量
SensorReader* MiniCloudProcesser::AngelReader = nullptr;
// 用静态变量定义旋转轴以及平移
static CCVector3 axis3dx(1, 0, 0);
static CCVector3 axis3dy(0, 1, 0);
static CCVector3 axis3dz(0, 0, 1);
static CCVector3 t3d(0, 0, 0);

MiniCloudProcesser::MiniCloudProcesser(ccPointCloud * pc, SensorReader* ar, QTextEdit *CSOutput, QString *CSTxt) {
	// 初始化成员变量
	tNode.t = nullptr;
	tNode.next = nullptr;
	ConsoleOutput = CSOutput;
	ConsoleTxt = CSTxt;
	currentPointNum = 0U;
	icpCounts = 0U;
	threadNum = 0U;
	model_pc = pc;
	AngelReader = ar;
	// 初始化临时点云
	data_pc.addScalarField("DefaultScalarField");
	data_pc.enableScalarField();
	data_pc.showSF(true);
	data_pc.setCurrentDisplayedScalarField(0);

	Output2Console("MIniProcesser INITED----");
}

void MiniCloudProcesser::AddPoint(LivoxRawPoint * point_data){
	if (point_data) {
		AddPoint(point_data->x, point_data->y, point_data->z, point_data->reflectivity);
	}
}

void MiniCloudProcesser::AddPoint(int32_t x, int32_t y, int32_t z, int32_t r) {
	//Output2Console(QString("currentPointNum %1 EEE %2").arg(currentPointNum).arg(miniCloudSize));
	// 在同一帧内添加点 - 增量片段
	if (currentPointNum < miniCloudSize) {
		ccPointCloud tmpCloud;
		tmpCloud.reserve(1);
		tmpCloud.addPoint(CCVector3(x, y, z));
		// xyz轴分别旋转
		ccGLMatrix matrx;
		ccGLMatrix matry;
		ccGLMatrix matrz;
		matrx.initFromParameters(AngelReader->rx, axis3dx, t3d);
		matry.initFromParameters(AngelReader->ry, axis3dy, t3d);
		matrz.initFromParameters(AngelReader->rz, axis3dz, t3d);
		// 应用旋转
		tmpCloud.rotateGL(matrx);
		tmpCloud.rotateGL(matry);
		tmpCloud.rotateGL(matrz);
		tmpCloud.applyGLTransformation_recursive();

		// 将旋转后的点添加到主体点云中去
		data_pc.reserve(1);
		data_pc.addPoint(CCVector3(tmpCloud.getPoint(0)->x, tmpCloud.getPoint(0)->y, tmpCloud.getPoint(0)->z));
		data_pc.getScalarField(0)->addElement(r);

		// 当前点云(临时点云)大小+1
		currentPointNum++;
	}
	else { // 临时点云大小满足一帧时,创建一个子线程完成ICP配准
		ThreadNode *tn;
		tn = (ThreadNode*)malloc(sizeof(ThreadNode));
		tn->next = tNode.next;
		tNode.next = tn;
		tn->t = new std::thread(&MiniCloudProcesser::ActICPTrans, this, data_pc);
		threadNum++;
		// 清空临时点云
		ResetDataPc();
		// 重复添加点云的操作
		AddPoint( x, y, z, r);
	}
}

void MiniCloudProcesser::ActICPTrans(ccPointCloud data_pc_full){
	if (icpCounts >= 2U) {
		//data_pc_full.getScalarField(0)->computeMinAndMax();
		//data_pc_full = SORFilter(data_pc_full);

		//ccPointCloud mdpc = SORFilter(*model_pc);



		// 变换矩阵 临时点云最终大小 最终缩放 最小均方根
		ccGLMatrix transMat;
		unsigned int finalPointCount;
		double finalScale, finalRMS;

		// SubSample

		bool status = 
			ccRegistrationTools::ICP(&data_pc_full,	  // 临时点云
					//&privous_pc,	  // 主体点云 - 上一帧
				model_pc,	        // 主体点云 - 全部

					transMat,		  // 返回一个变换矩阵
					finalScale,		  // 返回缩放值(不使用)
					finalRMS,		  // 返回最小均方根(不使用)
					finalPointCount,  // 返回最终点数
					0.000001,		  // 最小均方根衰减 
					20,				  // 最大迭代次数
					10000,			  // 随机下采样上限
					true,			  // 是否移除最远的点
					CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ERROR_CONVERGENCE,
					false,		      // 是否缩放
					0.95,			  // 最终重合率
					false,			  // 使用临时点云SF
					false,			  // 使用主体点云SF
					CCCoreLib::ICPRegistrationTools::SKIP_NONE, // 默认不过滤任何轴
					4,                // 最大线程数
					0);

		Output2Console(QString("Mat Status %1").arg((int)status));
		// 如果配准成功
		if (status) {
			// 执行变换
			data_pc_full.rotateGL(transMat);
			data_pc_full.applyGLTransformation_recursive();
			// 应用到主体点云上
			mtx.lock();
			for (unsigned int i = 0; i < finalPointCount; i++) {
				model_pc->addPoint(CCVector3(data_pc_full.getPoint(i)->x, data_pc_full.getPoint(i)->y, data_pc_full.getPoint(i)->z));
				model_pc->getScalarField(0)->addElement(data_pc_full.getScalarField(0)->getValue(i));
			}
			model_pc->getScalarField(0)->computeMinAndMax();
			mtx.unlock();
			// 转存临时点云作为前一帧
			SavePrivousPc(data_pc_full);
			icpCounts++;
		}
	}
	else {
		// 转存临时点云作为前一帧
		SavePrivousPc(data_pc_full);
		// 应用到主体点云上
		mtx.lock();
		for (unsigned int i = 0; i < miniCloudSize - 1; i++) {
			model_pc->reserve(1);
			model_pc->addPoint(CCVector3(data_pc_full.getPoint(i)->x, data_pc_full.getPoint(i)->y, data_pc_full.getPoint(i)->z));
			model_pc->getScalarField(0)->addElement(data_pc_full.getScalarField(0)->getValue(i));
		}
		model_pc->getScalarField(0)->computeMinAndMax();
		mtx.unlock();
		// ICP计数器自增1
		icpCounts++;
	}
}

void MiniCloudProcesser::ResetPointCloud(ccPointCloud & pc){
	pc.clear();
	pc.clearFWFData();
	pc.clearLOD();
	pc.deleteAllScalarFields();
	pc.deleteOctree();
	pc.removeAllChildren();
	pc.removeAllClipPlanes();
	pc.removeGrids();
	pc.addScalarField("DefaultScalarField");
	pc.enableScalarField();
	pc.showSF(true);
	pc.setCurrentDisplayedScalarField(0);
}

void MiniCloudProcesser::ResetDataPc() {
	ResetPointCloud(data_pc);
	currentPointNum = 0;
}

void MiniCloudProcesser::SayHello(int32_t x){
	Output2Console(QString("Hello %1").arg(x));
}

ccPointCloud MiniCloudProcesser::SORFilter(ccPointCloud data_pc_full){
	CCCoreLib::ReferenceCloud* selection = CCCoreLib::CloudSamplingTools::sorFilter(&data_pc_full, 6, 1.0);
	if (selection && &data_pc_full) {
		if (selection->size() == data_pc_full.size()) {
			Output2Console("A21321");
			ccLog::Warning(QString("[doActionSORFilter] No points were removed from cloud "));
			return data_pc_full;
		}
		else {
			ccPointCloud* cleanCloud = data_pc_full.partialClone(selection);
			if (cleanCloud) {
				Output2Console("Clean Successful");
				return *cleanCloud;
				//return data_pc_full;
			}
			else {
				Output2Console("NO CleanCloud");
				return data_pc_full;
			}
		}
	}
	else {
		Output2Console("NOT Filt");
		return data_pc_full;
	}
}

void MiniCloudProcesser::SavePrivousPc(ccPointCloud & from){
	mtx.lock();
	// 清空前一帧点云
	ResetPointCloud(privous_pc);
	// 转存
	for (unsigned int i=0;i<from.size();i++) {
		privous_pc.reserve(1);
		privous_pc.addPoint(CCVector3(from.getPoint(i)->x, from.getPoint(i)->y, from.getPoint(i)->z));
		privous_pc.getScalarField(0)->addElement(from.getScalarField(0)->getValue(i));
	}
	mtx.unlock();
}

void MiniCloudProcesser::Output2Console(QString *Txt) {
	ConsoleTxt->append("\n");
	ConsoleTxt->append(*Txt);
}

void MiniCloudProcesser::Output2Console(QString Txt) {
	Output2Console(&Txt);
}
