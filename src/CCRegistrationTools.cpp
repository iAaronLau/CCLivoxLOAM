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

// 在临时mesh上进行下采样的默认点数
static const unsigned s_defaultSampledPointsOnDataMesh = 50000;
// 默认的临时SF名称
static const char REGISTRATION_DISTS_SF[] = "RegistrationDistances";

bool ccRegistrationTools::ICP(	ccHObject* data,		      // 临时点云
								ccHObject* model,			  // 主体点云
								ccGLMatrix& transMat,		  // 返回一个变换矩阵
								double &finalScale,			  // 返回缩放值(不使用)
								double& finalRMS,			  // 返回最小均方根(不使用)
								unsigned& finalPointCount,	  // 返回最终点数
								double minRMSDecrease/* = 0.00001*/,	  // 最小均方根衰减 
								unsigned maxIterationCount/* = 20*/,      // 最大迭代次数
								unsigned randomSamplingLimit/* = 2000*/,  // 随机下采样上限
								bool removeFarthestPoints/* = true*/,     // 是否移除最远的点
								CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE method/* = CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ITER_CONVERGENCE*/,
								bool adjustScale/* = false*/,             // 是否缩放
								double finalOverlapRatio/* = 0.9*/,       // 最终重合率
								bool useDataSFAsWeights/* = true*/,       // 使用临时点云SF
								bool useModelSFAsWeights/* = true*/,      // 使用主体点云SF
								int filters/* = CCCoreLib::ICPRegistrationTools::SKIP_NONE*/, // 默认不过滤任何轴
								int maxThreadCount/* = 4*/,               // 最大线程数
								QWidget* parent/* = 0*/)               
{
	bool restoreColorState = false; // 还原颜色状态
	bool restoreSFState = false;    // 还原SF状态

	//进度条
	/*QScopedPointer<ccProgressDialog> progressDlg;
	if (parent){
		progressDlg.reset(new ccProgressDialog(false, parent));
	}*/

	CCCoreLib::Garbage<CCCoreLib::GenericIndexedCloudPersist> cloudGarbage;

	//如果model是mesh，必须先进行点采样
	CCCoreLib::GenericIndexedCloudPersist* modelCloud = nullptr;
	ccGenericMesh* modelMesh = nullptr;
	if (model->isKindOf(CC_TYPES::MESH)){
		modelMesh = ccHObjectCaster::ToGenericMesh(model);
		modelCloud = modelMesh->getAssociatedCloud();
	}
	else{
		modelCloud = ccHObjectCaster::ToGenericPointCloud(model);
	}

	//如果data是mesh，必须先进行点采样
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

	//启用一个临时SF 来进行配准距离计算
	CCCoreLib::ScalarField* dataDisplayedSF = nullptr;
	int oldDataSfIdx = -1;
	int dataSfIdx = -1;

	//如果data是一个真实的ccPointCloud，创建一个合适的临时 SF 来进行配准距离计算
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

	//在输入的比率中添加一个 安全边缘(safety margin) 
	static double s_overlapMarginRatio = 0.2;
	finalOverlapRatio = std::max(finalOverlapRatio, 0.01); // 最小0.01
	// 减小输入的点云，这样可以使点数接近重合点数的理论值，但也别太少点，可能会配准失败
	if (finalOverlapRatio < 1.0 - s_overlapMarginRatio){
		//DGM we can now use 'approximate' distances as SAITO algorithm is exact (but with a coarse resolution)
		//现在可以使用近似距离，但是会降低解析度
		//level = 7 if < 1.000.000
		//level = 8 if < 10.000.000
		//level = 9 if > 10.000.000
		//gridLevel = octree深度
		int gridLevel = static_cast<int>(floor(log10(static_cast<double>(std::max(dataCloud->size(), modelCloud->size()))))) + 2;
		    gridLevel = std::min(std::max(gridLevel, 7), 9); // gridLevel最大值9，最小值2
		int result = -1;
		if (modelMesh){ // 如果model是mesh
			CCCoreLib::DistanceComputationTools::Cloud2MeshDistanceComputationParams c2mParams;
			c2mParams.octreeLevel = gridLevel;
			c2mParams.maxSearchDist = 0;
			c2mParams.useDistanceMap = true;
			c2mParams.signedDistances = false;
			c2mParams.flipNormals = false;
			c2mParams.multiThread = false;
			result = CCCoreLib::DistanceComputationTools::computeCloud2MeshDistance(dataCloud, modelMesh, c2mParams);
		}
		else{// 如果model是点云
			result = CCCoreLib::DistanceComputationTools::computeApproxCloud2CloudDistance(	dataCloud,
																						modelCloud,
																						gridLevel,
																						-1);
		}

		if (result < 0){
			ccLog::Error("Failed to determine the max (overlap) distance (not enough memory?)");
			return false;
		}

		//测定最大距离，这个距离大致上与重合率相关
		ScalarType maxSearchDist = 0;
		{
			unsigned count = dataCloud->size();
			// distances里面装的就是ScalarValue，最后进行个ParallelSort
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
			
			// 在finalOverlapRatio + margin百分比的范围内找到最大值
			maxSearchDist = distances[static_cast<size_t>(std::max(1.0,count*(finalOverlapRatio+s_overlapMarginRatio)))-1];
		}

		//最终，选择距离小于'maxSearchDist'的点
		//应该大致上与finalOverlapRatio + margin百分比相关
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

			//更新 finalOverlapRatio
			finalOverlapRatio /= keptRatio;
		}
	}

	//weights SF加权
	CCCoreLib::ScalarField* modelWeights = nullptr;
	CCCoreLib::ScalarField* dataWeights = nullptr;
	{
		// 如果model不是mesh，并且使用SF权重
		if (!modelMesh && useModelSFAsWeights){
			if (modelCloud == dynamic_cast<CCCoreLib::GenericIndexedCloudPersist*>(model) && model->isA(CC_TYPES::POINT_CLOUD)){
				ccPointCloud* pc = static_cast<ccPointCloud*>(model);
				modelWeights = pc->getCurrentDisplayedScalarField();//获取SF
				if (!modelWeights)
					ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but model has no displayed scalar field!");
			}
			else{
				ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but only point cloud scalar fields can be used as weights!");
			}
		}
		// 如果data使用SF权重
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

	// 准备完数据，正式开始ICP
	CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;       //转换结果
	CCCoreLib::PointProjectionTools::Transformation transform; //转换XX？
	CCCoreLib::ICPRegistrationTools::Parameters params;        //ICP参数
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
	//转换失败
	if (result >= CCCoreLib::ICPRegistrationTools::ICP_ERROR){
		ccLog::Error("Registration failed: an error occurred (code %i)",result);
	}
	//转换成功
	else if (result == CCCoreLib::ICPRegistrationTools::ICP_APPLY_TRANSFO){
		transMat = FromCCLibMatrix<PointCoordinateType, float>(transform.R, transform.T, transform.s);
		finalScale = transform.s;
	}

	//删除临时的SF (如果有的话)
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
