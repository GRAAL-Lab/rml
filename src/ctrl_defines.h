/*
 * ctrl_defines.h
 *
 *  Created on: Jun 22, 2011
 *      Author: enrico
 */

#ifndef __CTRL_DEFINES_H__
#define __CTRL_DEFINES_H__

#define ITERATIVE_TIMING_SUPPORT_ENABLED

#define CTRL_SIZE_STATE_NAME               (32)
#define CTRL_SIZE_COMMAND_NAME             (32)
#define CTRL_SIZE_CONSOLE_COMMAND          (15)
#define CTRL_SIZE_CONSOLE_PARAMLIST        (40)
#define CTRL_SIZE_CONSOLE_DESCRIPTION      (255)

#define CTRL_BBS_CATEGORY                   "CTRL"

#define CTRL_MAX_ARMJOINTS                  (8)
#define CTRL_MAX_VEHICLEDOF                 (6)

#define CTRL_MODEL_UNDEFINED                (-1)
#define CTRL_MODEL_BACKWARDFORWARD          (1)

#define CTRL_COMMAND_UNKNOWN                (-1)
#define CTRL_COMMAND_ARM                    (1)
#define CTRL_COMMAND_HOLD                   (2)
#define CTRL_COMMAND_ENABLE                 (3)
#define CTRL_COMMAND_PARK                   (4)
#define CTRL_COMMAND_EXITPARK               (5)
#define CTRL_COMMAND_JPOSCTRL               (6)
#define CTRL_COMMAND_JVELCTRL               (7)
#define CTRL_COMMAND_CPOSCTRL               (8)
#define CTRL_COMMAND_CVELCTRL               (9)
#define CTRL_COMMAND_RELOADPAR              (10)
#define CTRL_COMMAND_QUIT                   (11)
#define CTRL_COMMAND_DISARM                 (12)
#define CTRL_COMMAND_SHOWPAR                (13)
#define CTRL_COMMAND_FIRSTAVL               (100)

#define CTRL_STATE_UNKNOWN                  (-1)
#define CTRL_STATE_INIT                     (0)
#define CTRL_STATE_DISARMED                 (1)
#define CTRL_STATE_JPOSCTRL                 (2)
#define CTRL_STATE_HOLDING                  (3)
#define CTRL_STATE_PARKING                  (4)
#define CTRL_STATE_EXITING                  (5)
#define CTRL_STATE_FAILURE                  (6)
#define CTRL_STATE_JVELCTRL                 (7)
#define CTRL_STATE_CVELCTRL                 (8)
#define	CTRL_STATE_CPOSCTRL                 (9)
#define	CTRL_STATE_QUIT                     (10)
#define CTRL_STATE_APPROACHING              (11)
#define CTRL_STATE_FIRSTAVL                 (100)

#define	CTRL_RV_WORKING                     (3)
#define	CTRL_RV_NEWVALUE                    (2)
#define CTRL_RV_OK                          (1)
#define CTRL_RV_FAIL                        (-1)
#define CTRL_RV_EEOR                        (-2)
#define CTRL_RV_ENONEWVALUE                 (-3)
#define CTRL_RV_EINVALIDPAR                 (-4)
#define CTRL_RV_ETEMPORARYDISABLED          (-5)
#define CTRL_RV_EALREADYEXISTS              (-6)
#define CTRL_RV_ENOBLOCK                    (-7)
#define CTRL_RV_ETIMEOUT                    (-8)

#define CTRL_CMDRV_CMDOK                    (1)
#define CTRL_CMDRV_CMDFAIL                  (-1)
#define CTRL_CMDRV_CMDUNKNOWN               (-2)
#define CTRL_CMDRV_CMDFORBIDDEN             (-3)
#define CTRL_CMDRV_CMDUNASSOCIATED          (-4)
#define CTRL_CMDRV_UNKNOWNRV                (-5) // return value deleted by another command that comes later

#define CTRL_TIMEOUT_DISABLED               (-1)

#define CTRL_SVDPARAM_TIKHONOV              (0)
#define CTRL_SVDPARAM_W                     (1)
#define CTRL_SVDPARAM_PINV                  (2)
#define CTRL_MAX_PINV_PARAMS                (4)

#include <vector>
#include <cmath>
//#include <cmat/cmat.h>

#include "futils.h"

namespace CTRL
{
/**
 * @brief The CtrlMode enum defines the various modes in which an arm can be
 * controlled and has to be set externally.
 */
enum class CtrlMode
	: int16_t
	{
		Unknown, Velocity, Torque
};
/*
 struct DataInformation {
 int64_t      lastOperationTime_;
 char         bbsName_[BBS_DATA_NAME_SIZE];
 unsigned int size_;
 };
 */
struct Command
{
	Command(void)
	{
		command_ = CTRL_COMMAND_UNKNOWN;
	}
	int command_;
};
/*
 struct CommandWithName {
 CommandWithName(void) {
 command_ = CTRL_COMMAND_UNKNOWN;
 taskName_[0] = '\0';
 }
 int	command_;
 char taskName_[WF_TNAME_SIZE];
 };
 */
struct CommandReturnValue
{
	CommandReturnValue(void)
	{
		command_ = CTRL_COMMAND_UNKNOWN;
		returnValue_ = CTRL_CMDRV_UNKNOWNRV;
	}
	int command_;
	int returnValue_;
};
/*
 struct CommandReturnValueWithName {
 CommandReturnValueWithName(void) {
 command_ = CTRL_COMMAND_UNKNOWN;
 returnValue_ = CTRL_CMDRV_UNKNOWNRV;
 taskName_[0] = '\0';
 }
 int	command_;
 int	returnValue_;
 char taskName_[WF_TNAME_SIZE];
 };
 */
struct CartesianPositionReference
{
	double reference[6];
	bool enable[6];
};

struct CartesianVelocityReference
{
	double reference[6];
	bool enable[6];
};

struct CartesianGains
{
	double gain[6];
};

struct CartesianPositionFeedback
{
	double x[6];
};

struct JointPositionReference
{
	double reference[CTRL_MAX_ARMJOINTS];
	bool enable[CTRL_MAX_ARMJOINTS];
};

struct JointVelocityReference
{
	double reference[CTRL_MAX_ARMJOINTS];
	bool enable[CTRL_MAX_ARMJOINTS];
};

struct JointVelocityDriverReference
{
	double reference[CTRL_MAX_ARMJOINTS];
	double cartref[CTRL_MAX_ARMJOINTS];
};

struct JointGains
{
	double gain[CTRL_MAX_ARMJOINTS];
};

struct JointPositionFeedback
{
	double q[CTRL_MAX_ARMJOINTS];
};

struct JointEndOfRaces
{
	double high[CTRL_MAX_ARMJOINTS];
	double low[CTRL_MAX_ARMJOINTS];
};

struct JointPresetPosition
{
	double q[CTRL_MAX_ARMJOINTS];
};

struct ArmParameters
{
	int cartesianAlgorithm_;
	int iterativeNumberOfIterations_;
	double jointPositionErrorThreshold_;
	double jointVelocitySaturation_;
	double cartesianLinearVelocitySaturation_;
	double cartesianAngularVelocitySaturation_;
	double cartesianLinearPositionErrorThreshold_;
	double cartesianAngularPositionErrorThreshold_;
	double virtualFrameGain_;
	double jacobianThreshold_;
	double jacobianLambda_;
	double jointVelocityFilter_[2];
	long long jointVelocityTimeout_;
	long long cartesianVelocityTimeout_;
	bool enableJointVelocityFilter_;
	bool enableCartesianVelocityFilter_;
	bool disableManipulabilityTask_;
	bool enableEndOfRaces_;
	JointEndOfRaces jointEndOfRacesUserDefined_;
	JointEndOfRaces jointEndOfRacesActive_;
	JointEndOfRaces jointEndOfRacesDefault_;
	JointPresetPosition jointParkedPosition_;
	JointPresetPosition jointExitedPosition_;
};

struct Status
{
	int status_;
};

enum Frame
{
	frame_goal, frame_base
};
enum ReferenceType
{
	reftype_absolute, reftype_relative
};

struct CRefHelperPar
{
	Frame frame;
	Frame relativeTo;
	ReferenceType type;
};

struct CartesianParameters
{
	CRefHelperPar trasl;
	CRefHelperPar rot;
};


/// F.W. Additions ///

///**
// * @brief The CtrlMode enum defines the various modes in which the arm can be
// * controlled and has to be set externally.
// */
//enum class CtrlMode
//	: int16_t
//	{
//		Unknown, Velocity, Torque
//};

const std::map<CtrlMode, std::string> ctrlMode2Str { { CtrlMode::Unknown, "Unknown" }, { CtrlMode::Velocity, "Velocity" }, {
		CtrlMode::Torque, "Torque" } };

struct ArmParams
{
	int numJoints;
	double gamma;
	double GAMMA_def, GAMMA_wvf, QDOT_max, QDOTDOT_max, PHI, TORQUE_max;
	double qErrThresh, angErrThresh, linErrThresh;
	int errThreshDuration;
	bool virtualGoalEnable;
	double virt_vMax, virt_distExp;
	bool taskPriorityActive;
	bool driverActive;
	bool holdingActive;
	CtrlMode controlMode;

	std::vector<double> jointLimMax;
	std::vector<double> jointLimMin;
	std::vector<double> jointVelGain;
	std::vector<double> kp_gain_inner, ki_gain_inner;

	ArmParams() :
			gamma(0.0), GAMMA_def(2.0), GAMMA_wvf(2.0),	PHI(10), qErrThresh(0.001), angErrThresh(4 * M_PI / 180.0),
			linErrThresh(0.01), errThreshDuration(5), QDOT_max(0.5), QDOTDOT_max(0.5), TORQUE_max(10.0),
			virtualGoalEnable(true), virt_vMax(0.1), virt_distExp(0.1), numJoints(0), taskPriorityActive(true),
			driverActive(true), holdingActive(false), controlMode(CtrlMode::Velocity)
	{}

	void Init(int numberOfJoints) {

		numJoints = numberOfJoints;
		jointLimMax.resize(numJoints, 0.0);
		jointLimMin.resize(numJoints, 0.0);
		jointVelGain.resize(numJoints, 0.0);
		ki_gain_inner.resize(numJoints, 0.0);
		kp_gain_inner.resize(numJoints, 0.0);
	}

	friend std::ostream& operator <<(std::ostream& os, ArmParams const& a)
	{
		return os  << 	"TaskPriorityActive: " << a.taskPriorityActive << "\n" <<
						"DriverActive: " << a.driverActive << "\n" <<
						"HoldingActive: " << a.holdingActive << "\n" <<
						"ControlMode: " << ctrlMode2Str.at(a.controlMode) << "\n" <<
						"Gains:\n" <<
						"\tgamma: " << a.gamma << "\n" <<
						"\tGAMMA_def: " << a.GAMMA_def << "\n" <<
						"\tGAMMA_wvf: " << a.GAMMA_wvf << "\n" <<
						"\tPHI: " << a.PHI << "\n" <<
						"\tkp_gain_inner: " << FUTILS::STLVectorToString(a.kp_gain_inner, ',') << "\n" <<
						"\tki_gain_inner: " << FUTILS::STLVectorToString(a.ki_gain_inner, ',') <<  "\n" <<
						"Thresholds:" << "\n" <<
						"\tQDOT_max: " << a.QDOT_max << "\n" <<
						"\tQDOTDOT_max: " << a.QDOTDOT_max << "\n" <<
						"\tTORQUE_max: " << a.TORQUE_max << "\n" <<
						"\tqErrThresh: " << a.qErrThresh << "\n" <<
						"\tangErrThresh: " << a.angErrThresh << "\n" <<
						"\tlinErrThresh: " << a.linErrThresh << "\n" <<
						"\terrThreshDuration: " << a.errThreshDuration << "\n" <<
						"\tvirtualGoalEnable: " << a.virtualGoalEnable << "\n" <<
						"\tvirt_vMax: " << a.virt_vMax << "\n" <<
						"\tvirt_distExp: " << a.virt_distExp << "\n";


	}
};

/**
 * @brief Exception to be thrown when the arm index is outside bounds
 */
class ArmIndexException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "Wrong arm index!";
	}
};

}

#endif /* CTRL_DEFINES_H_ */
