#ifndef __RMLEXCEPTIONS_H__
#define __RMLEXCEPTIONS_H__

#include <iostream>
#include <memory>
namespace rml
{

class ExceptionWithID: public std::exception
{
public:

    void SetID(std::string ID)
    {
        ID_ = ID;
    }
    const char* who() const throw ()
    {
        return ID_.c_str();
    }
private:
    std::string ID_;
};

/// ROBOT MODEL
/**
 * @brief Exception to be thrown when the joint index out of bounds
 */
class RobotModelArmException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: Not existing arm model !!!";
    }
};

/**
 * @brief Exception to be thrown when vehicle is not present
 */
class RobotModelVehicleException : public std::exception {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: No vehicle!!!";
    }
};

/**
 * @brief Exception to be thrown when  the matrix label is wrong
 */
class RobotModelWrongLabelException : public std::exception {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: wrong label for getJacobian";
    }
};

/**
 * @brief Exception to be thrown when load a not initialized arm model
 */
class RobotModelNotInitializedArmModelException : public std::exception {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: loaded a not initialized arm model";
    }
};

/**
 * @brief Exception to be thrown when load a not initialized arm model
 */
class RobotModelConflictingArmModelIDException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: existing arm model with same id ";
    }
};

/**
 * @brief Exception to be thrown when load a not initialized vehicle model
 */
class RobotModelNotInitializedVehicleModelException : public std::exception {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: loaded a not initialized vehicle model";
    }
};

///ARM MODEL
/**
 * @brief Exception to be thrown when setting joint variable on a not initialized arm model
 */
class ArmModelNotInitializedException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: setting variable on a not initialized model ";
    }
};

/**
 * @brief Exception to be thrown when setting joint variable of wrong size
 */
class ArmModelWrongJointSizeException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: setting joint variable of wrong size ";
    }
};

/**
 * @brief Exception to be thrown when trying to access a not existing joint
 */
class ArmModelNotExistingJointException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: trying to acess a not existing joint";
    }
};

/**
 * @brief Exception to be thrown when trying to access a not existing joint
 */
class LabelAlreadyUsedException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: Label for frame already associated to another joint";
    }
};


/**
 * @brief Exception to be thrown when wrong label in input
 */
class ArmModelWrongLabelException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[ArmModel] Error: wrong format input string";
    }
};

///VEHICLE MODEL
/**
 * @brief Exception to be thrown when wrong label in input
 */
class VehicleModelWrongLabelException : public ExceptionWithID {
    virtual const char* what() const throw()
    {
        return "[VehicleModel] Error: wrong format input string";
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
