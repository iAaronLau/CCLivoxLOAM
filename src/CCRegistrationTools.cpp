#include "CCRegistrationTools.h"

//CCCoreLib
#include <CloudSamplingTools.h>
#include <DistanceComputationTools.h>
#include <Garbage.h>
#include <GenericIndexedCloudPersist.h>
#include <MeshSamplingTools.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <RegistrationTools.h>

//qCC_db
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

#include <set>

// ����ʱmesh�Ͻ����²�����Ĭ�ϵ���
static const unsigned s_defaultSampledPointsOnDataMesh = 50000;
// Ĭ�ϵ���ʱSF����
static const char REGISTRATION_DISTS_SF[] = "RegistrationDistances";

bool ccRegistrationTools::ICP(	ccHObject* data,		      // ��ʱ����
								ccHObject* model,			  // �������
								ccGLMatrix& transMat,		  // ����һ���任����
								double &finalScale,			  // ��������ֵ(��ʹ��)
								double& finalRMS,			  // ������С������(��ʹ��)
								unsigned& finalPointCount,	  // �������յ���
								double minRMSDecrease/* = 0.00001*/,	  // ��С������˥�� 
								unsigned maxIterationCount/* = 20*/,      // ����������
								unsigned randomSamplingLimit/* = 2000*/,  // ����²�������
								bool removeFarthestPoints/* = true*/,     // �Ƿ��Ƴ���Զ�ĵ�
								CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE method/* = CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ITER_CONVERGENCE*/,
								bool adjustScale/* = false*/,             // �Ƿ�����
								double finalOverlapRatio/* = 0.9*/,       // �����غ���
								bool useDataSFAsWeights/* = true*/,       // ʹ����ʱ����SF
								bool useModelSFAsWeights/* = true*/,      // ʹ���������SF
								int filters/* = CCCoreLib::ICPRegistrationTools::SKIP_NONE*/, // Ĭ�ϲ������κ���
								int maxThreadCount/* = 4*/,               // ����߳���
								QWidget* parent/* = 0*/)               
{
	bool restoreColorState = false; // ��ԭ��ɫ״̬
	bool restoreSFState = false;    // ��ԭSF״̬

	//������
	/*QScopedPointer<ccProgressDialog> progressDlg;
	if (parent){
		progressDlg.reset(new ccProgressDialog(false, parent));
	}*/

	CCCoreLib::Garbage<CCCoreLib::GenericIndexedCloudPersist> cloudGarbage;

	//���model��mesh�������Ƚ��е����
	CCCoreLib::GenericIndexedCloudPersist* modelCloud = nullptr;
	ccGenericMesh* modelMesh = nullptr;
	if (model->isKindOf(CC_TYPES::MESH)){
		modelMesh = ccHObjectCaster::ToGenericMesh(model);
		modelCloud = modelMesh->getAssociatedCloud();
	}
	else{
		modelCloud = ccHObjectCaster::ToGenericPointCloud(model);
	}

	//���data��mesh�������Ƚ��е����
	CCCoreLib::GenericIndexedCloudPersist* dataCloud = nullptr;
	if (data->isKindOf(CC_TYPES::MESH)){
		dataCloud = CCCoreLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(data), s_defaultSampledPointsOnDataMesh);
		if (!dataCloud){
			ccLog::Error("[ICP] Failed to sample points on 'data' mesh!");
			return false;
		}
		cloudGarbage.add(dataCloud);
	}
	else{
		dataCloud = ccHObjectCaster::ToGenericPointCloud(data);
	}

	//����һ����ʱSF ��������׼�������
	CCCoreLib::ScalarField* dataDisplayedSF = nullptr;
	int oldDataSfIdx = -1;
	int dataSfIdx = -1;

	//���data��һ����ʵ��ccPointCloud������һ�����ʵ���ʱ SF ��������׼�������
	if (data->isA(CC_TYPES::POINT_CLOUD)){
		ccPointCloud* pc = static_cast<ccPointCloud*>(data);
		restoreColorState = pc->colorsShown();
		restoreSFState = pc->sfShown();
		dataDisplayedSF = pc->getCurrentDisplayedScalarField();
		oldDataSfIdx = pc->getCurrentInScalarFieldIndex();
		dataSfIdx = pc->getScalarFieldIndexByName(REGISTRATION_DISTS_SF);
		if (dataSfIdx < 0)
			dataSfIdx = pc->addScalarField(REGISTRATION_DISTS_SF);
		if (dataSfIdx >= 0)
			pc->setCurrentScalarField(dataSfIdx);
		else{
			ccLog::Error("[ICP] Couldn't create temporary scalar field! Not enough memory?");
			return false;
		}
	}
	else{
		if (!dataCloud->enableScalarField()){
			ccLog::Error("[ICP] Couldn't create temporary scalar field! Not enough memory?");
			return false;
		}
	}

	//������ı��������һ�� ��ȫ��Ե(safety margin) 
	static double s_overlapMarginRatio = 0.2;
	finalOverlapRatio = std::max(finalOverlapRatio, 0.01); // ��С0.01
	// ��С����ĵ��ƣ���������ʹ�����ӽ��غϵ���������ֵ����Ҳ��̫�ٵ㣬���ܻ���׼ʧ��
	if (finalOverlapRatio < 1.0 - s_overlapMarginRatio){
		//DGM we can now use 'approximate' distances as SAITO algorithm is exact (but with a coarse resolution)
		//���ڿ���ʹ�ý��ƾ��룬���ǻή�ͽ�����
		//level = 7 if < 1.000.000
		//level = 8 if < 10.000.000
		//level = 9 if > 10.000.000
		//gridLevel = octree���
		int gridLevel = static_cast<int>(floor(log10(static_cast<double>(std::max(dataCloud->size(), modelCloud->size()))))) + 2;
		    gridLevel = std::min(std::max(gridLevel, 7), 9); // gridLevel���ֵ9����Сֵ2
		int result = -1;
		if (modelMesh){ // ���model��mesh
			CCCoreLib::DistanceComputationTools::Cloud2MeshDistanceComputationParams c2mParams;
			c2mParams.octreeLevel = gridLevel;
			c2mParams.maxSearchDist = 0;
			c2mParams.useDistanceMap = true;
			c2mParams.signedDistances = false;
			c2mParams.flipNormals = false;
			c2mParams.multiThread = false;
			result = CCCoreLib::DistanceComputationTools::computeCloud2MeshDistance(dataCloud, modelMesh, c2mParams);
		}
		else{// ���model�ǵ���
			result = CCCoreLib::DistanceComputationTools::computeApproxCloud2CloudDistance(	dataCloud,
																						modelCloud,
																						gridLevel,
																						-1);
		}

		if (result < 0){
			ccLog::Error("Failed to determine the max (overlap) distance (not enough memory?)");
			return false;
		}

		//�ⶨ�����룬���������������غ������
		ScalarType maxSearchDist = 0;
		{
			unsigned count = dataCloud->size();
			// distances����װ�ľ���ScalarValue�������и�ParallelSort
			std::vector<ScalarType> distances; 
			try{
				distances.resize(count);
			}
			catch (const std::bad_alloc&){
				ccLog::Error("Not enough memory!");
				return false;
			}
			for (unsigned i=0; i<count; ++i){
				distances[i] = dataCloud->getPointScalarValue(i);
			}
			
			ParallelSort(distances.begin(), distances.end());
			
			// ��finalOverlapRatio + margin�ٷֱȵķ�Χ���ҵ����ֵ
			maxSearchDist = distances[static_cast<size_t>(std::max(1.0,count*(finalOverlapRatio+s_overlapMarginRatio)))-1];
		}

		//���գ�ѡ�����С��'maxSearchDist'�ĵ�
		//Ӧ�ô�������finalOverlapRatio + margin�ٷֱ����
		{
			CCCoreLib::ReferenceCloud* refCloud = new CCCoreLib::ReferenceCloud(dataCloud);
			cloudGarbage.add(refCloud);
			unsigned countBefore = dataCloud->size();
			unsigned baseIncrement = static_cast<unsigned>(std::max(100.0,countBefore*finalOverlapRatio*0.05));
			for (unsigned i=0; i<countBefore; ++i)
			{
				if (dataCloud->getPointScalarValue(i) <= maxSearchDist)
				{
					if (	refCloud->size() == refCloud->capacity()
						&&	!refCloud->reserve(refCloud->size() + baseIncrement) )
					{
						ccLog::Error("Not enough memory!");
						return false;
					}
					refCloud->addPointIndex(i);
				}
			}
			refCloud->resize(refCloud->size());
			dataCloud = refCloud;

			unsigned countAfter = dataCloud->size();
			double keptRatio = static_cast<double>(countAfter)/countBefore;
			ccLog::Print(QString("[ICP][Partial overlap] Selecting %1 points out of %2 (%3%) for registration").arg(countAfter).arg(countBefore).arg(static_cast<int>(100*keptRatio)));

			//���� finalOverlapRatio
			finalOverlapRatio /= keptRatio;
		}
	}

	//weights SF��Ȩ
	CCCoreLib::ScalarField* modelWeights = nullptr;
	CCCoreLib::ScalarField* dataWeights = nullptr;
	{
		// ���model����mesh������ʹ��SFȨ��
		if (!modelMesh && useModelSFAsWeights){
			if (modelCloud == dynamic_cast<CCCoreLib::GenericIndexedCloudPersist*>(model) && model->isA(CC_TYPES::POINT_CLOUD)){
				ccPointCloud* pc = static_cast<ccPointCloud*>(model);
				modelWeights = pc->getCurrentDisplayedScalarField();//��ȡSF
				if (!modelWeights)
					ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but model has no displayed scalar field!");
			}
			else{
				ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but only point cloud scalar fields can be used as weights!");
			}
		}
		// ���dataʹ��SFȨ��
		if (useDataSFAsWeights){
			if (!dataDisplayedSF){
				if (dataCloud == ccHObjectCaster::ToPointCloud(data))
					ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but data has no displayed scalar field!");
				else
					ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but only point cloud scalar fields can be used as weights!");
			}
			else{
				dataWeights = dataDisplayedSF;
			}
		}
	}

	// ׼�������ݣ���ʽ��ʼICP
	CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;       //ת�����
	CCCoreLib::PointProjectionTools::Transformation transform; //ת��XX��
	CCCoreLib::ICPRegistrationTools::Parameters params;        //ICP����
	{
		params.convType = method;
		params.minRMSDecrease = minRMSDecrease;
		params.nbMaxIterations = maxIterationCount;
		params.adjustScale = adjustScale;
		params.filterOutFarthestPoints = removeFarthestPoints;
		params.samplingLimit = randomSamplingLimit;
		params.finalOverlapRatio = finalOverlapRatio;
		params.modelWeights = modelWeights;
		params.dataWeights = dataWeights;
		params.transformationFilters = filters;
		params.maxThreadCount = maxThreadCount;
	}

	result = CCCoreLib::ICPRegistrationTools::Register(	modelCloud,
													modelMesh,
													dataCloud,
													params,
													transform,
													finalRMS,
													finalPointCount);
	//ת��ʧ��
	if (result >= CCCoreLib::ICPRegistrationTools::ICP_ERROR){
		ccLog::Error("Registration failed: an error occurred (code %i)",result);
	}
	//ת���ɹ�
	else if (result == CCCoreLib::ICPRegistrationTools::ICP_APPLY_TRANSFO){
		transMat = FromCCLibMatrix<PointCoordinateType, float>(transform.R, transform.T, transform.s);
		finalScale = transform.s;
	}

	//ɾ����ʱ��SF (����еĻ�)
	if (dataSfIdx >= 0)
	{
		assert(data->isA(CC_TYPES::POINT_CLOUD));
		ccPointCloud* pc = static_cast<ccPointCloud*>(data);
		pc->setCurrentScalarField(oldDataSfIdx);
		pc->deleteScalarField(dataSfIdx);
		pc->showColors(restoreColorState);
		pc->showSF(restoreSFState);
	}

	return (result < CCCoreLib::ICPRegistrationTools::ICP_ERROR);
}
