#ifndef CC_REGISTRATION_TOOLS_HEADER
#define CC_REGISTRATION_TOOLS_HEADER

#include <RegistrationTools.h>
#include <ccGLMatrix.h>

class QWidget;
class QStringList;
class ccHObject;

class ccRegistrationTools{
public:

	// ���������ƶ��������׼ - ICP
	// ���������mesh ���Զ������²���
	static bool ICP(ccHObject* data,		      // ��ʱ����
					ccHObject* model,			  // �������
					ccGLMatrix& transMat,		  // ����һ���任����
					double &finalScale,			  // ��������ֵ(��ʹ��)
					double& finalRMS,			  // ������С������(��ʹ��)
					unsigned& finalPointCount,	  // �������յ���
					double minRMSDecrease = 0.00001,	 // ��С������˥�� 
					unsigned maxIterationCount = 20,     // ����������
					unsigned randomSamplingLimit = 2000, // ����²�������
					bool removeFarthestPoints = true,    // �Ƿ��Ƴ���Զ�ĵ�
					CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE method = CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ERROR_CONVERGENCE,
					bool adjustScale = false,            // �Ƿ�����
					double finalOverlapRatio = 0.95,     // �����غ���
					bool useDataSFAsWeights = false,     // ʹ����ʱ����SF
					bool useModelSFAsWeights = false,    // ʹ���������SF
					int filters = CCCoreLib::ICPRegistrationTools::SKIP_NONE, // Ĭ�ϲ������κ���
					int maxThreadCount = 4,              // ����߳���
					QWidget* parent = 0);

};

#endif 
