#include "MiniCloudProcesser.h"
#include "CCRegistrationTools.h"
#include <CloudSamplingTools.h>

// ��ʼ����̬����
SensorReader* MiniCloudProcesser::AngelReader = nullptr;
// �þ�̬����������ת���Լ�ƽ��
static CCVector3 axis3dx(1, 0, 0);
static CCVector3 axis3dy(0, 1, 0);
static CCVector3 axis3dz(0, 0, 1);
static CCVector3 t3d(0, 0, 0);

MiniCloudProcesser::MiniCloudProcesser(ccPointCloud * pc, SensorReader* ar, QTextEdit *CSOutput, QString *CSTxt) {
	// ��ʼ����Ա����
	tNode.t = nullptr;
	tNode.next = nullptr;
	ConsoleOutput = CSOutput;
	ConsoleTxt = CSTxt;
	currentPointNum = 0U;
	icpCounts = 0U;
	threadNum = 0U;
	model_pc = pc;
	AngelReader = ar;
	// ��ʼ����ʱ����
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
	// ��ͬһ֡����ӵ� - ����Ƭ��
	if (currentPointNum < miniCloudSize) {
		ccPointCloud tmpCloud;
		tmpCloud.reserve(1);
		tmpCloud.addPoint(CCVector3(x, y, z));
		// xyz��ֱ���ת
		ccGLMatrix matrx;
		ccGLMatrix matry;
		ccGLMatrix matrz;
		matrx.initFromParameters(AngelReader->rx, axis3dx, t3d);
		matry.initFromParameters(AngelReader->ry, axis3dy, t3d);
		matrz.initFromParameters(AngelReader->rz, axis3dz, t3d);
		// Ӧ����ת
		tmpCloud.rotateGL(matrx);
		tmpCloud.rotateGL(matry);
		tmpCloud.rotateGL(matrz);
		tmpCloud.applyGLTransformation_recursive();

		// ����ת��ĵ���ӵ����������ȥ
		data_pc.reserve(1);
		data_pc.addPoint(CCVector3(tmpCloud.getPoint(0)->x, tmpCloud.getPoint(0)->y, tmpCloud.getPoint(0)->z));
		data_pc.getScalarField(0)->addElement(r);

		// ��ǰ����(��ʱ����)��С+1
		currentPointNum++;
	}
	else { // ��ʱ���ƴ�С����һ֡ʱ,����һ�����߳����ICP��׼
		ThreadNode *tn;
		tn = (ThreadNode*)malloc(sizeof(ThreadNode));
		tn->next = tNode.next;
		tNode.next = tn;
		tn->t = new std::thread(&MiniCloudProcesser::ActICPTrans, this, data_pc);
		threadNum++;
		// �����ʱ����
		ResetDataPc();
		// �ظ���ӵ��ƵĲ���
		AddPoint( x, y, z, r);
	}
}

void MiniCloudProcesser::ActICPTrans(ccPointCloud data_pc_full){
	if (icpCounts >= 2U) {
		//data_pc_full.getScalarField(0)->computeMinAndMax();
		//data_pc_full = SORFilter(data_pc_full);

		//ccPointCloud mdpc = SORFilter(*model_pc);



		// �任���� ��ʱ�������մ�С �������� ��С������
		ccGLMatrix transMat;
		unsigned int finalPointCount;
		double finalScale, finalRMS;

		// SubSample

		bool status = 
			ccRegistrationTools::ICP(&data_pc_full,	  // ��ʱ����
					//&privous_pc,	  // ������� - ��һ֡
				model_pc,	        // ������� - ȫ��

					transMat,		  // ����һ���任����
					finalScale,		  // ��������ֵ(��ʹ��)
					finalRMS,		  // ������С������(��ʹ��)
					finalPointCount,  // �������յ���
					0.000001,		  // ��С������˥�� 
					20,				  // ����������
					10000,			  // ����²�������
					true,			  // �Ƿ��Ƴ���Զ�ĵ�
					CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ERROR_CONVERGENCE,
					false,		      // �Ƿ�����
					0.95,			  // �����غ���
					false,			  // ʹ����ʱ����SF
					false,			  // ʹ���������SF
					CCCoreLib::ICPRegistrationTools::SKIP_NONE, // Ĭ�ϲ������κ���
					4,                // ����߳���
					0);

		Output2Console(QString("Mat Status %1").arg((int)status));
		// �����׼�ɹ�
		if (status) {
			// ִ�б任
			data_pc_full.rotateGL(transMat);
			data_pc_full.applyGLTransformation_recursive();
			// Ӧ�õ����������
			mtx.lock();
			for (unsigned int i = 0; i < finalPointCount; i++) {
				model_pc->addPoint(CCVector3(data_pc_full.getPoint(i)->x, data_pc_full.getPoint(i)->y, data_pc_full.getPoint(i)->z));
				model_pc->getScalarField(0)->addElement(data_pc_full.getScalarField(0)->getValue(i));
			}
			model_pc->getScalarField(0)->computeMinAndMax();
			mtx.unlock();
			// ת����ʱ������Ϊǰһ֡
			SavePrivousPc(data_pc_full);
			icpCounts++;
		}
	}
	else {
		// ת����ʱ������Ϊǰһ֡
		SavePrivousPc(data_pc_full);
		// Ӧ�õ����������
		mtx.lock();
		for (unsigned int i = 0; i < miniCloudSize - 1; i++) {
			model_pc->reserve(1);
			model_pc->addPoint(CCVector3(data_pc_full.getPoint(i)->x, data_pc_full.getPoint(i)->y, data_pc_full.getPoint(i)->z));
			model_pc->getScalarField(0)->addElement(data_pc_full.getScalarField(0)->getValue(i));
		}
		model_pc->getScalarField(0)->computeMinAndMax();
		mtx.unlock();
		// ICP����������1
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
	// ���ǰһ֡����
	ResetPointCloud(privous_pc);
	// ת��
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
