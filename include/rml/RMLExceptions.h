#ifndef __RMLEXCEPTIONS_H__
#define __RMLEXCEPTIONS_H__

#include <iostream>
#include <memory>
namespace rml {
class ExceptionWithHow : public std::exception {
public:
    void SetHow(std::string how)
    {
        how_ = how;
    }
    const char* how() const noexcept
    {
        return how_.c_str();
    }

private:
    std::string how_;
};

// GENERAL
class LabelSyntaxException : public ExceptionWithHow {
    virtual const char* what() const noexcept
    {
        return "[RML] Label Syntax Error! (check how())";
    }
};

// ROBOT MODEL
/**
 * @brief Exception to be thrown in robot model when dealing with the arm model
 */
class RobotModelArmException : public ExceptionWithHow {
    virtual const char* what() const noexcept
    {
        return "[RobotModel] Error: Arm model exception (check how())";
    }
};

/**
 * @brief Exception to be thrown when trying to set a wrong control vector
 */
class RobotModelWrongControlSizeVectorException : public std::exception {
    virtual const char* what() const noexcept
    {
        return "[RobotModel] Error: wrong input size vector (check how())";
    }
};

/**
 * @brief Exception to be thrown in robot model when dealing with wrong frame id's
 */
class WrongFrameException : public ExceptionWithHow {
    virtual const char* what() const noexcept
    {
        return "Error: wrong frame id (check how())";
    }
};

// ARM MODEL
/**
 * @brief Exception to be thrown when setting joint variable on a not initialized arm model
 */
class ArmModelNotInitializedException : public std::exception {
    virtual const char* what() const noexcept
    {
        return "[ArmModel] Error: setting variable on a not initialized model ";
    }
};

/**
 * @brief Exception to be thrown when setting joint variable of wrong size
 */
class ArmModelJointException : public ExceptionWithHow {
    virtual const char* what() const noexcept
    {
        return "[ArmModel] Error: trying to access wrong joints (Check how()) ";
    }
};

/**
 * @brief Exception to be thrown when the vehicle model is not initialized
 */
class VehicleModelNotInitializedException : public std::exception {
    virtual const char* what() const noexcept
    {
        return "[VehicleModel] Error: vehicle model not initialized ";
    }
};

///FUNCTIONS
/**
 * @brief Exception to be thrown id wrong initialization of bell shape parameters
 */
class BellShapeParameterException : public ExceptionWithHow {
    virtual const char* what() const noexcept
    {
        return "[rml::Functions] Error: wrong bell shape parameters. (Check how())";
    }
};

/**
 * @brief Exception to be thrown id wrong initialization of bell shape parameters
 */
//class BellShapeParameterException: public std::exception {
//    virtual const char* what() const throw()
//    {
//        return "[rml::Functions] Error: wrong bell shape parameters: ymin > ymax";
//    }
//};
}

#endif
