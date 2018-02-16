/**
 * @file ctrl_task_defines.h
 *
 *  Created on: Mar 14, 2016
 *      @author: fwanderlingh
 */

#ifndef CTRL_TASK_DEFINES_H_
#define CTRL_TASK_DEFINES_H_

#ifndef CMAT_STANDALONE
#define CMAT_STANDALONE
#endif

#include <cmath>
#include <cmat/cmat.h>

#include "control_baxter/ctrl_defines.h"

#define CTRL_LOG_SIZE_APLHA
#define CTRL_LOG_SIZE_FLAG
#define CTRL_LOG_SIZE_MU

#define MAX_CONTROLLER_TASKS    (10)
#define MAX_TASK_SIZE		  (10)

/*#define TASK_MANIPULABILITY           (0)
 #define TASK_JOINT_LIMITS             (1)
 #define TASK_EE_LINEAR                (3)
 #define TASK_EE_ANGULAR               (4)
 #define TASK_VEHICLE_DISTANCE		  (5)
 */
const int max_dof = 20;
const int max_ctrl_tasks = 10;
const int max_task_size = 10;
const int max_task_name_size = 32;

namespace CTRL
{

#define SVDPARAM_TIKHONOV 0
#define SVDPARAM_W 1
#define SVDPARAM_PINV 2
#define BELLTYPE_NONE 0
#define BELLTYPE_INC 1
#define BELLTYPE_DEC 2
#define BELLTYPE_BOTH 3
#define ARM_INDEX 0
#define VEHICLE_INDEX 1
#define CENTRALIZED_INDEX 2
#define MAX_PINV_PARAMS 4

/**
 * Remeber to change also the taskNames!!
 */
enum TaskType
{
	MANIPULABILITY, JOINT_LIMITS, EE_LINEAR, EE_ANGULAR, COORDINATION, ELBOW_DISTANCE, ELBOW_ANGLE, enumSize
};

const std::map<TaskType,std::string> TaskNames = {
		{TaskType::MANIPULABILITY, "MANIPULABILITY"},
		{TaskType::JOINT_LIMITS, "JOINT_LIMITS"},
		{TaskType::EE_LINEAR, "EE_LINEAR"},
		{TaskType::EE_ANGULAR, "EE_ANGULAR"},
		{TaskType::COORDINATION, "COORDINATION"},
        {TaskType::ELBOW_ANGLE, "ELBOW_ANGLE"},
        {TaskType::ELBOW_DISTANCE, "ELBOW_DISTANCE"}
};

enum ConstraintType
{
	Undefined, Equality, Inequality
};

struct TaskEvolution
{
	double sigma[MAX_TASK_SIZE];
	double sigmaDot[MAX_TASK_SIZE];
	double sigmaDotBar[MAX_TASK_SIZE];
	double sigmaIntegral[MAX_TASK_SIZE];
};

struct ReferenceParameters
{
	double gain;
	double saturation;
};

/**
 * This structure contains all the pseudo inverse parameters
 * There are different matrices that are inverted
 * SVDPARAM_TIKHONOV: the values refer to those used to compute the regularization matrix used in the algorithm
 * SVDPARAM_W: the values are used to compute the pseudo inverse for the W calculation
 * SVDPARAM_PINV: the values are used to compute the regularized pseudo inverse of J (as per paper)
 */

/*
 struct SVDInversionParameters
 {
 double xidotmax;
 double thetadomax;
 };
*/

struct SVDParameters
{
	double threshold; 	// the value below which the raised cosine becomes > 0
	double lambda;    	// the maximum value of the raised cosine
	double mu;
	int flag;
};

struct SVDSingularValues
{
	double mu;     ///< the product of all singular values
	int flag;      ///< indicates how many singular values are zero
};

struct BellShapedFunctionParameters
{
	double sigma1;
	double sigma2;
};

struct PseudoInverseParameters
{
	SVDSingularValues svdvalues[3];
	SVDParameters svdparams;
};

/**
 *
 * Here all the parameters relative to the TaskPriority
 *
 */

struct ControllerTaskParameters
{
	ReferenceParameters ref;
	SVDParameters svd;
	int belltype;
	CTRL::BellShapedFunctionParameters bell[2];

	ControllerTaskParameters() :
			belltype(BELLTYPE_NONE)
	{
		bell[0].sigma1 = 0.0;
		bell[0].sigma2 = 0.0;
		bell[1].sigma1 = 0.0;
		bell[1].sigma2 = 0.0;
	}
};

struct ControllerParameters
{
	ControllerTaskParameters tasks[MAX_CONTROLLER_TASKS];
	//double o_alignAxis[3];
	//double vTc_vector[6];
	double vTb_vector[6];
	double eTt_vector[6];
	bool enableVehicle;
	//bool enableCameraCenteringTask;
	//bool useVehicleLinearFbk;
	//bool useVehicleAngularFbk;
	//char pad[2];
};

struct PrecisionTaskParameters
{
	PseudoInverseParameters svd;
};

struct SingleShapedSetTaskParameters
{
	PseudoInverseParameters svd;
	BellShapedFunctionParameters bell;
};

struct DoubleShapedSetTaskParameters
{
	PseudoInverseParameters svd;
	BellShapedFunctionParameters decreasingBell;
	BellShapedFunctionParameters increasingBell;
};

struct ArmControllerTaskParameters
{
	PrecisionTaskParameters prEEApproach;
	SingleShapedSetTaskParameters srJointLimits;
	SingleShapedSetTaskParameters srManipulability;
	ReferenceParameters prRefEEApproach;
	ReferenceParameters srRefJointLimits;
	ReferenceParameters srRefManipulability;
};

struct TaskFeedback
{
	int dimension;
	bool enabled;
	SVDParameters svdparams[CTRL_MAX_PINV_PARAMS];
	double xdot[10];
	double alpha[10];
};

struct TaskControlFeedback
{
	TaskFeedback jointLimits;
	TaskFeedback manipulability;
	TaskFeedback eeApproach;
};

struct TaskWork
{
	int dimension;
	CMAT::Matrix A;
	CMAT::Matrix s;
	CMAT::Matrix n;
	CMAT::Matrix Ja;
	CMAT::Matrix Jv;
	CMAT::Matrix Ga;	 			// n^T Ja oppure Ja
	CMAT::Matrix Gv; 				// n^T Jv oppure Jv
	CMAT::Matrix reference; 		// velocity reference for the task
	int jacobianDimension;
	//int belltype;
	int belltype; //specifies which kind of bell (INCREASING, DECREASING, OR BOTH) should be used
	bool useVersor;
	CTRL::BellShapedFunctionParameters bell[2]; // activation specifications for the centralized approach
};

struct Task
{
	int dimension;
	CMAT::Matrix Ga;        		// n^T Ja oppure Ja
	CMAT::Matrix Gv;        		// n^T Jv oppure Jv
	CMAT::Matrix rho;
	CMAT::Matrix v;
	CMAT::Matrix reference; 		// the task reference
	CMAT::Matrix A;      			// the activation matrix
	CMAT::Matrix y; 				// the system velocity generated for this task (Centralized version only)

	CTRL::PseudoInverseParameters pinvPar[3];

	bool enabled;
	ConstraintType constrType;

	Task() : enabled(false), dimension(0), constrType(Undefined) {}
};

struct TaskLog
{
	TaskType type;
	int taskSize;
	int dof;
	double rho[max_dof];
	double y[max_dof];
	double alpha[max_task_size];
	bool enable;
};

/*
struct TaskArmLog
{
	int dimension;
	double rho[CTRL_MAX_ARMJOINTS];
	CTRL::PseudoInverseParameters pinvPar[3];
	double y[CTRL_MAX_ARMJOINTS];
	double alpha[MAX_TASK_SIZE];
	TaskEvolution evol;
};
*/
}

#endif /* CTRL_TASK_DEFINES_H_ */

