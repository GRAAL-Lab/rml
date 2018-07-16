#ifndef __RMLEXCEPTIONS_H__
#define __RMLEXCEPTIONS_H__

#include <iostream>
#include <memory>
namespace rml
{

class ExceptionWithIDandMethod: public std::exception
{
public:

    void SetWho(std::string ID)
    {
        ID_ = ID;
    }
    const char* who() const throw ()
    {
        return ID_.c_str();
    }
    void SetWhere(std::string method)
    {
        method_ = method;
    }
    const char* where() const throw ()
    {
        return method_.c_str();
    }

private:
    std::string ID_;
    std::string method_;
};

class ExceptionWithHow: public std::exception
{
public:

    void SetHow(std::string how)
    {
        how_ = how;
    }
    const char* how() const throw ()
    {
        return how_.c_str();
    }

private:
    std::string how_;
};

/// ROBOT MODEL
/**
 * @brief Exception to be thrown in robot model when dealing with the arm model
 */
class RobotModelArmException : public ExceptionWithHow{
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: Arm model exception";
    }
};


/**
 * @brief Exception to be thrown in robot model when dealing with the vehicle model
 */
class RobotModelVehicleException : public ExceptionWithHow {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: Vehicle Exception";
    }
};


/**
 * @brief Exception to be thrown when trying to set a wrong control vector
 */
class RobotModelWrongControlSizeVectorException : public std::exception {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: wrong input size vector";
    }
};


/**
 * @brief Exception to be thrown in robot model when dealing with wrong frame id's
 */
class RobotModelWrongFrameException : public ExceptionWithHow {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: wrong frame id";
    }
};


///ARM MODEL
/**
 * @brief Exception to be thrown when setting joint variable on a not initialized arm model
 */
class ArmModelNotInitializedException : public ExceptionWithIDandMethod {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: setting variable on a not initialized model ";
    }
};

/**
 * @brief Exception to be thrown when setting joint variable of wrong size
 */
class ArmModelWrongJointSizeException : public ExceptionWithIDandMethod {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: setting joint variable of wrong size ";
    }
};

/**
 * @brief Exception to be thrown when trying to access a not existing joint
 */
class ArmModelNotExistingJointException : public ExceptionWithIDandMethod {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: trying to acess a not existing joint";
    }
};

/**
 * @brief Exception to be thrown when trying to access a not existing joint
 */
class LabelAlreadyUsedException : public ExceptionWithIDandMethod {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: Label for frame already associated to another joint";
    }
};


/**
 * @brief Exception to be thrown when wrong label in input
 */
class ArmModelWrongLabelException : public ExceptionWithIDandMethod {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: The frame does not exist";
    }
};

///VEHICLE MODEL
/**
 * @brief Exception to be thrown when wrong label in input
 */
class VehicleModelWrongLabelException : public ExceptionWithIDandMethod {
    virtual const char* what() const throw()
    {
        return "[VehicleModel] Error: The frame does not exist";
    }
};

/**
 * @brief Exception to be thrown when the vehicle model is not initialized
 */
class VehicleModelNotInitializedException: public std::exception {
    virtual const char* what() const throw()
    {
        return "[VehicleModel] Error: vehicle model not initialized ";
    }
};

///FUNCTIONS
/**
 * @brief Exception to be thrown id wrong initialization of bell shape parameters
 */
class FunctionBellShapeParameterException: public std::exception {
    virtual const char* what() const throw()
    {
        return "[rml::Functions] Error: wrong bell shape parameters: xmin > xmax";
    }
};

/**
 * @brief Exception to be thrown id wrong initialization of bell shape parameters
 */
class FunctionBellShapeYException: public std::exception {
    virtual const char* what() const throw()
    {
        return "[rml::Functions] Error: wrong bell shape parameters: ymin > ymax";
    }
};

}

#endif
