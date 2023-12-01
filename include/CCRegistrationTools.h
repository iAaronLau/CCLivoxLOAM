#ifndef CC_REGISTRATION_TOOLS_HEADER
#define CC_REGISTRATION_TOOLS_HEADER

#include <RegistrationTools.h>
#include <ccGLMatrix.h>

class QWidget;
class QStringList;
class ccHObject;

class ccRegistrationTools{
public:

	// 用两个点云对象进行配准 - ICP
	// 如果传入了mesh 会自动对其下采样
	static bool ICP(ccHObject* data,		      // 临时点云
					ccHObject* model,			  // 主体点云
					ccGLMatrix& transMat,		  // 返回一个变换矩阵
					double &finalScale,			  // 返回缩放值(不使用)
					double& finalRMS,			  // 返回最小均方根(不使用)
					unsigned& finalPointCount,	  // 返回最终点数
					double minRMSDecrease = 0.00001,	 // 最小均方根衰减 
					unsigned maxIterationCount = 20,     // 最大迭代次数
					unsigned randomSamplingLimit = 2000, // 随机下采样上限
					bool removeFarthestPoints = true,    // 是否移除最远的点
					CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE method = CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ERROR_CONVERGENCE,
					bool adjustScale = false,            // 是否缩放
					double finalOverlapRatio = 0.95,     // 最终重合率
					bool useDataSFAsWeights = false,     // 使用临时点云SF
					bool useModelSFAsWeights = false,    // 使用主体点云SF
					int filters = CCCoreLib::ICPRegistrationTools::SKIP_NONE, // 默认不过滤任何轴
					int maxThreadCount = 4,              // 最大线程数
					QWidget* parent = 0);

};

#endif 
