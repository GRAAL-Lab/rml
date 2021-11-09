/*
 * ctrl_armModel.cc
 *
 *  Created on: Jul 25, 2015
 *      Author: francescow
 */

#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdlib.h>
#include <string>
#include <utility>
#include <vector>

#include "ArmModel.h"
#include "Functions.h"
#include "MatrixOperations.h"
#include "PseudoInverse.h"
#include "RMLDefines.h"
#include "RMLExceptions.h"
#include "RobotLink.h"
#include "SVD.h"
#include "Types.h"
#include "rml_internal/Futils.h"

using std::cout;
using std::endl;

namespace rml {

ArmModel::ArmModel(const std::string id) noexcept(false)
    : modelInitialized_(false)
    , isMapInitialized_(false)
    , totalNumJoints_(0)
    , modelReadFromFile_(false)
{
    std::size_t underscorepos = id.find_first_of("_");
    if (underscorepos == std::string::npos) {
        id_ = std::move(id);
    } else {
        LabelSyntaxException labelException;
        labelException.SetHow("ArmModel() constructor: Underscores '_' not allowed in ID");
        throw(labelException);
    }
}

ArmModel::~ArmModel()
{
}

void ArmModel::AddJointLink(JointType type, const Eigen::Vector3d& axis, const Eigen::TransformationMatrix& baseTransf, double jointLimMin, double joinLimMax)
{
    links_.push_back(RobotLink(type, axis, baseTransf, jointLimMin, joinLimMax));

    totalNumJoints_ = static_cast<unsigned int>(links_.size());
    movingJoints_.push_back(totalNumJoints_ - 1);
    movingNumJoints_ = static_cast<unsigned int>(movingJoints_.size());

    baseTei_.push_back(Eigen::TransformationMatrix());
    biTei_.push_back(Eigen::TransformationMatrix());
    h_.push_back(Eigen::Vector6d());
    dJdq_.push_back(Eigen::MatrixXd());
    for (auto&& i : dJdq_) { // access by forwarding reference, the type of i is auto&
        i.resize(6, totalNumJoints_);
        i.setZero();
    }

    q_total_ = Eigen::VectorXd::Zero(totalNumJoints_);

    q_moving_ = Eigen::VectorXd::Zero(movingNumJoints_);
    q_dot_moving_ = Eigen::VectorXd::Zero(movingNumJoints_);
    q_ddot_moving_ = Eigen::VectorXd::Zero(movingNumJoints_);
    controlRef_ = Eigen::VectorXd::Zero(movingNumJoints_);

    modelInitialized_ = true;
    isMapInitialized_ = false;
    JointsPosition(Eigen::VectorXd::Zero(totalNumJoints_));
}

void ArmModel::AddFixedLink(const Eigen::TransformationMatrix& baseTransf)
{
    links_.push_back(RobotLink(JointType::Fixed, Eigen::Vector3d::UnitZ(), baseTransf, 0, 0));
    totalNumJoints_ = static_cast<unsigned int>(links_.size());

    baseTei_.push_back(Eigen::TransformationMatrix());
    biTei_.push_back(Eigen::TransformationMatrix());
    h_.push_back(Eigen::Vector6d());
    dJdq_.push_back(Eigen::MatrixXd());
    for (auto&& i : dJdq_) { // access by forwarding reference, the type of i is auto&
        i.resize(6, totalNumJoints_);
        i.setZero();
    }

    q_total_ = Eigen::VectorXd::Zero(totalNumJoints_);

    modelInitialized_ = true;
    isMapInitialized_ = false;
    JointsPosition(Eigen::VectorXd::Zero(movingNumJoints_));
}

void ArmModel::JointsPosition(const Eigen::VectorXd fbk) noexcept(false)
{
    if (!modelInitialized_) {
        ArmModelNotInitializedException armModelNotIntialized;
        throw(armModelNotIntialized);
    }

    if (fbk.size() != movingNumJoints_) {
        ArmModelJointException armModelWrongJointSize;
        throw(armModelWrongJointSize);
    }

    for (unsigned int i = 0; i < movingJoints_.size(); i++) {
        q_total_(movingJoints_.at(i)) = fbk(i);
    }
    q_moving_ = std::move(fbk);

    EvaluatedJdqNumeric();
    EvaluateTotalForwardGeometry();

    if (!isMapInitialized_) {
        transformation_.erase(transformation_.begin(), transformation_.end());
        jacobians_.erase(jacobians_.begin(), jacobians_.end());
        //updating the joint jacobians
        for (unsigned int i = 0; i < totalNumJoints_; i++) {
            transformation_.insert(std::make_pair(id_ + FrameID::Joint + std::to_string(i), baseTei_.at(i)));
            jacobians_.insert(std::make_pair(id_ + FrameID::Joint + std::to_string(i), EvaluateBase2JointJacobian(i)));
        }
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, IndexedTMat>::iterator iter = rigidBodyFrames_.begin(); iter != rigidBodyFrames_.end(); ++iter) {
            std::string id = iter->first;
            transformation_.insert(std::make_pair(id, EvaluateRigidBodyTransf(id)));
            jacobians_.insert(std::make_pair(id, EvaluateRigidBodyJacobian(id)));
        }
        isMapInitialized_ = true;
    } else {
        //updating the joint jacobians
        for (unsigned int i = 0; i < totalNumJoints_; i++) {
            transformation_.find(id_ + FrameID::Joint + std::to_string(i))->second = baseTei_.at(i);
            jacobians_.find(id_ + FrameID::Joint + std::to_string(i))->second = EvaluateBase2JointJacobian(i);
        }
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, IndexedTMat>::iterator iter = rigidBodyFrames_.begin(); iter != rigidBodyFrames_.end(); ++iter) {
            std::string id = iter->first;
            transformation_.find(id)->second = EvaluateRigidBodyTransf(id);
            jacobians_.find(id)->second = EvaluateRigidBodyJacobian(id);
        }
    }
    manipulability_.erase(manipulability_.begin(), manipulability_.end());
    manipulabilityJacobians_.erase(manipulabilityJacobians_.begin(), manipulabilityJacobians_.end());
    dexterity_.erase(dexterity_.begin(), dexterity_.end());
    dexterityJacobians_.erase(dexterityJacobians_.begin(), dexterityJacobians_.end());
}

void ArmModel::JointsVelocity(const Eigen::VectorXd qdot) noexcept(false)
{
    if (!modelInitialized_) {
        ArmModelNotInitializedException armModelNotIntialized;
        throw(armModelNotIntialized);
    }
    if (qdot.size() != q_dot_moving_.size()) {
        ArmModelJointException armModelWrongJointSize;
        std::string how = "Wrong input size vector in set joints velocity ";
        armModelWrongJointSize.SetHow(how);
        throw(armModelWrongJointSize);
    }
    q_dot_moving_ = std::move(qdot);
}

void ArmModel::JointsAcceleration(const Eigen::VectorXd qddot) noexcept(false)
{
    if (!modelInitialized_) {
        ArmModelNotInitializedException armModelNotIntialized;
        throw(armModelNotIntialized);
    }
    if (qddot.size() != q_ddot_moving_.size()) {
        ArmModelJointException armModelWrongJointSize;
        std::string how = "Wrong input size vector in set joints acceleration";
        armModelWrongJointSize.SetHow(how);
        throw(armModelWrongJointSize);
    }
    q_ddot_moving_ = std::move(qddot);
}

void ArmModel::EvaluateTotalForwardGeometry()
{
    for (unsigned int jointNumber = 0; jointNumber < totalNumJoints_; jointNumber++) {
        ForwardDirectGeometry(jointNumber);
    }
}

void ArmModel::ForwardDirectGeometry(unsigned int jointNumber)
{
    Tz_ = Eigen::TransformationMatrix();
    // wTbi is the transformation between the base of joint <jointNumber> and the base frame <b>
    if (jointNumber == 0) {
        baseTbi_ = Eigen::TransformationMatrix();
    } else {
        // in this case, the base of joint <jointNumber> is the position of the end-effector of the joint <jointNumber - 1>
        baseTbi_ = baseTei_.at(jointNumber - 1);
    }

    // biTri is the constant transformation between the base of the joint <i> and its end-effector
    // biTei also takes into account the actual rotation of the joint, so biTei = biTri * Tz(qi)
    if (links_.at(jointNumber).Type() == JointType::Revolute) {
        Eigen::AngleAxisd rot = Eigen::AngleAxisd(q_total_(jointNumber), links_.at(jointNumber).Axis());
        Tz_.RotationMatrix(rot.toRotationMatrix());
    } else if (links_.at(jointNumber).Type() == JointType::Prismatic) {
        Eigen::Vector3d transl = q_total_(jointNumber) * links_.at(jointNumber).Axis();
        Tz_.TranslationVector(transl);
    } else if (links_.at(jointNumber).Type() == JointType::Fixed) {
        Tz_.setIdentity();
    }

    // biTei = biTri * Tz(qi)
    biTei_.at(jointNumber) = links_.at(jointNumber).BaseTransf() * Tz_;

    // baseTei_ is the transformation between the end-effector of joint <i> and base frame <b> baseTei_ = wTbi * biTei
    baseTei_.at(jointNumber) = baseTbi_ * biTei_.at(jointNumber);
}

void ArmModel::BackwardDirectGeometry(unsigned int jointNumber, unsigned int endEffectorIndex)
{
    // Calcolo w_ki
    // Dal momento che ri_ki e ei_ki sono ruotate lungo ki e per convenzione ogni giunto ruota lungo z si ha che ri_ki = ei_ki = [ 0 0 1 ]'
    // Di conseguenza w_ki e' la 3a colonna della R che lo proietta sul mondo
    h_.at(jointNumber) = Eigen::VectorXd::Zero(6);
    if (links_.at(jointNumber).Type() == JointType::Revolute) {
        base_ki_ = baseTei_.at(jointNumber).block(0, 2, 3, 1); //GetSubMatrix(1, 3, 3, 3));
        h_.at(jointNumber).AngularVector(base_ki_);
        h_.at(jointNumber).LinearVector(base_ki_.cross(baseTei_.at(endEffectorIndex).TranslationVector() - baseTei_.at(jointNumber).TranslationVector()));
    } else if (links_.at(jointNumber).Type() == JointType::Prismatic) {
        base_ki_ = baseTei_.at(jointNumber).block(0, 2, 3, 1); //GetSubMatrix(1, 3, 3, 3)); 
        h_.at(jointNumber).AngularVector(Eigen::Vector3d(0,0,0));
        h_.at(jointNumber).LinearVector(base_ki_);
    } else if (links_.at(jointNumber).Type() == JointType::Fixed) {
        h_.at(jointNumber).AngularVector(Eigen::Vector3d(0,0,0));
        h_.at(jointNumber).LinearVector(Eigen::Vector3d(0,0,0));
    }
}

Eigen::MatrixXd ArmModel::EvaluateManipulability(const std::string& frameID)
{
    Eigen::MatrixXd J = Jacobian(frameID);
    long n = J.cols();

    Eigen::MatrixXd Jmu, Jpinv;

    Eigen::MatrixXd dJdq;

    Jmu.resize(1, n);
    Jmu.setZero();

    rml::RegularizationData mySVD;
    if (n < 6) {
        //TODO change svd data
        mySVD.params.lambda = 0.0;
        mySVD.params.threshold = 0.00;
        /// For defective manipulators
        Jpinv = rml::RegularizedPseudoInverse(static_cast<Eigen::MatrixXd>(J.transpose()), mySVD);
        for (unsigned int k = 0; k < n; k++) {
            Jmu(k) = 0;
            djdqJpinv_ = dJdq_.at(k).transpose() * Jpinv;

            for (int i = 0; i < n; i++)
                Jmu(k) += djdqJpinv_(i, i);

            Jmu(k) *= mySVD.results.mu;
        }
    } else {
        mySVD.params.lambda = 0.01;
        mySVD.params.threshold = 0.01;
        Jpinv = rml::RegularizedPseudoInverse(J, mySVD);

        for (unsigned int k = 0; k < n; k++) {
            Jmu(k) = 0;
            djdqJpinv_ = dJdq_.at(k) * Jpinv;
            for (int i = 0; i < 6; i++)
                Jmu(k) += djdqJpinv_(i, i);
            Jmu(k) *= mySVD.results.mu;
        }
    }

    manipulability_.insert(std::make_pair(frameID, mySVD.results.mu));

    return Jmu;
}

void ArmModel::EvaluatedJdqNumeric()
{
    Eigen::MatrixXd bJt_0, bJt_dQ;
    Eigen::MatrixXd dQ, qVar, q_orig;
    double delta_q = 1E-6;
    q_orig = q_total_;
    //bJt_0 = bJt_;
    EvaluateTotalForwardGeometry();
    bJt_0 = EvaluateBase2JointJacobian(movingJoints_.back());

    // Here we iterate till "numJoints - 1" since the last joint will not influence on the Jacobian since nothing is connected to it.
    for (unsigned int i = 0; i < totalNumJoints_ - 1; ++i) {

        // Computing the single q variation vector dQ.
        dQ = Eigen::VectorXd::Zero(totalNumJoints_); // Initialising a zero vector.

        if (links_.at(i).Type() != JointType::Fixed) {
            dQ(i) = delta_q; // Getting the q variation
        }

        qVar = q_orig + dQ; // Computing the new single q variation based on previous state

        // Computing scalar delta_q (the denominator of the numerical integration).
        q_total_ = qVar;

        EvaluateTotalForwardGeometry();

        //EvaluateBase2JointJacobian(numberOfJoints_ - 1);
        //bJt_dQ = bJt_;
        bJt_dQ = EvaluateBase2JointJacobian(movingJoints_.back());
        //futils::PrettyPrint(wJt_dQ,"wJt_dQ");
        for (int iJrow = 0; iJrow < 6; ++iJrow) {
            for (unsigned int iJcol = 0; iJcol < movingNumJoints_; ++iJcol) {
                dJdq_.at(i)(iJrow, iJcol) = (bJt_dQ(iJrow, iJcol) - bJt_0(iJrow, iJcol)) / delta_q;
                if (std::fabs(dJdq_.at(i)(iJrow, iJcol)) < 1E-6)
                    dJdq_.at(i)(iJrow, iJcol) = 0.0;
            }
        }
    }
    q_total_ = q_orig;

    //std::cout << "AFTER" << std::endl;
}

Eigen::MatrixXd ArmModel::EvaluateBase2JointJacobian(unsigned int jointIndex)
{
    //Here we calculate the h columns involved in our joint's jacobian
    for (int jointNumber = jointIndex; jointNumber >= 0; jointNumber--)
        BackwardDirectGeometry(jointNumber, jointIndex);

    //Then we set all the remainings columns to zero
    for (unsigned int i = jointIndex + 1; i < totalNumJoints_; i++) {
        h_.at(i).setZero();
    }

    Eigen::MatrixXd bJj; // = (h_[0]);
    for (unsigned int i = 0; i < movingNumJoints_; i++) {
        bJj = RightJuxtapose(bJj, h_.at(movingJoints_.at(i)));
    }

    return bJj;
}

void ArmModel::AttachRigidBodyFrame(std::string frameID, std::string attachedFrameID, Eigen::TransformationMatrix attachedFrameID_T_frameID) noexcept(false)
{
    std::string rigidBodyID = id_ + "_" + frameID;

    // Check if rigid body is already present
    if (rigidBodyFrames_.find(rigidBodyID) != rigidBodyFrames_.end()) {

        // Check if the associated joint is the same otherwise throw exception
        if (rigidBodyFrames_.at(rigidBodyID).first == attachedFrameID) {
            rigidBodyFrames_.at(rigidBodyID).second = attachedFrameID_T_frameID;
            transformation_.at(rigidBodyID) = EvaluateRigidBodyTransf(rigidBodyID);
            jacobians_.at(rigidBodyID) = EvaluateRigidBodyJacobian(rigidBodyID);
        } else {
            WrongFrameException except;
            std::string how = "[ARM MODEL] Trying to change attached frame for a rigid body " + frameID;
            except.SetHow(how);
            throw(except);
        }
    } else {
        IndexedTMat myMat(attachedFrameID, attachedFrameID_T_frameID);
        rigidBodyFrames_.insert(std::make_pair(rigidBodyID, myMat));
        transformation_.insert(std::make_pair(rigidBodyID, EvaluateRigidBodyTransf(rigidBodyID)));
        jacobians_.insert(std::make_pair(rigidBodyID, EvaluateRigidBodyJacobian(rigidBodyID)));
    }
}

Eigen::TransformationMatrix ArmModel::EvaluateRigidBodyTransf(const std::string& frameID)
{
    return transformation_.at(rigidBodyFrames_.at(frameID).first) * rigidBodyFrames_.at(frameID).second;
}

Eigen::MatrixXd ArmModel::EvaluateRigidBodyJacobian(const std::string& frameID)
{
    std::string attachedFrameID = rigidBodyFrames_.at(frameID).first;
    return RigidBodyMatrix(transformation_.at(attachedFrameID).RotationMatrix() * rigidBodyFrames_.at(frameID).second.TranslationVector()) * jacobians_.at(attachedFrameID);
}

Eigen::TransformationMatrix ArmModel::TransformationMatrix(const std::string& frameID) noexcept(false)
{
    if (transformation_.find(frameID) == transformation_.end()) {
        WrongFrameException armModelWrongLabel;
        std::string how = "[ARM MODEL] The frame does not exist " + frameID;
        armModelWrongLabel.SetHow(how);
        throw(armModelWrongLabel);
    }
    return transformation_.at(frameID);
}

Eigen::TransformationMatrix ArmModel::TransformationMatrix(const std::string& frameID_j, const std::string& frameID_k)
{
    return TransformationMatrix(frameID_j).transpose() * TransformationMatrix(frameID_k);
}

Eigen::MatrixXd ArmModel::Jacobian(const std::string& frameID) noexcept(false)
{
    if (jacobians_.find(frameID) == jacobians_.end()) {
        WrongFrameException armModelWrongLabel;
        std::string how = "[ARM MODEL] The frame does not exist " + frameID;
        armModelWrongLabel.SetHow(how);
        throw(armModelWrongLabel);
    }
    return jacobians_.at(frameID);
}

Eigen::MatrixXd ArmModel::ManipulabilityJacobian(const std::string& frameID)
{
    if (manipulabilityJacobians_.find(frameID) == manipulabilityJacobians_.end()) {
        manipulabilityJacobians_.insert(std::make_pair(frameID, EvaluateManipulability(frameID)));
    }
    return (manipulabilityJacobians_.at(frameID));
}

RobotLink& ArmModel::Link(int jointIndex) noexcept(false)
{
    if (jointIndex < static_cast<int>(links_.size()))
        return links_.at(static_cast<unsigned long>(jointIndex));
    else {
        ArmModelJointException armModelNotExistingJoint;
        std::string how = "Trying to acess a not existing joint in get link ";
        armModelNotExistingJoint.SetHow(how);
        throw(armModelNotExistingJoint);
    }
}

void ArmModel::ControlVector(const Eigen::VectorXd controlRef) noexcept(false)
{
    if (controlRef.rows() == movingNumJoints_) {
        controlRef_ = std::move(controlRef);
    } else {
        ArmModelJointException armModelWrongJointSize;
        std::string how = "Wrong size vector in Set control vector ";
        armModelWrongJointSize.SetHow(how);
        throw(armModelWrongJointSize);
    }
}

double ArmModel::Manipulability(const std::string& frameID)
{
    if (manipulability_.find(frameID) == manipulability_.end()) {
        std::cerr << "rml::ArmModel, Manipulability() no frame " << frameID << std::endl;
    }
    return manipulability_.at(frameID);
}
    
Eigen::MatrixXd ArmModel::EvaluateDexterity(const std::string& frameID)
{
    Eigen::MatrixXd J = Jacobian(frameID);
    long n = J.cols();

    Eigen::MatrixXd Jdext, Jpinv, dJnorm_dq, dJpinvnorm_dq;
    double dext;
    Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n,n);
    Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6,6);

    Eigen::MatrixXd dJdq;

    Jdext.resize(1, n);
    Jdext.setZero();
    dJnorm_dq.resize(1, n);
    dJnorm_dq.setZero();
    dJpinvnorm_dq.resize(1, n);
    dJpinvnorm_dq.setZero();

    rml::RegularizationData mySVD;
    if (n < 6) {
        //TODO change svd data
        mySVD.params.lambda = 0.0;
        mySVD.params.threshold = 0.00;
        /// For defective manipulators
        Jpinv = rml::RegularizedPseudoInverse(J, mySVD);
        dext = 1/(J.norm()*Jpinv.norm());

        for (unsigned int k = 0; k < n; k++) {
            dJdq = dJdq_.at(k);
            Jdext(k) = 0;
            Jdjdq_ = J.transpose() * dJdq;
            Jpinvdjpinvdq_ = Jpinv.transpose()*(-Jpinv*dJdq*Jpinv
                                                +Jpinv*Jpinv.transpose()*dJdq.transpose()*(In-J*Jpinv)
                                                +(In-Jpinv*J)*dJdq.transpose()*Jpinv.transpose()*Jpinv);
            for (int i = 0; i < n; i++) {
                dJnorm_dq(k) += Jdjdq_(i, i);
                dJpinvnorm_dq(k) += Jpinvdjpinvdq_(i, i);
            }
            dJnorm_dq(k) *= 1/J.norm();
            dJpinvnorm_dq(k) *= 1/Jpinv.norm();

            Jdext(k) = -pow(dext,2)*(dJnorm_dq(k)*Jpinv.norm()+J.norm()*dJpinvnorm_dq(k));
        }
    } else {
        mySVD.params.lambda = 0.01;
        mySVD.params.threshold = 0.01;
        Jpinv = rml::RegularizedPseudoInverse(J, mySVD);
        dext = 1/(J.norm()*Jpinv.norm());

        for (unsigned int k = 0; k < n; k++) {
            dJdq = dJdq_.at(k);
            Jdext(k) = 0;
            Jdjdq_ = J.transpose() * dJdq;
            Jpinvdjpinvdq_ = Jpinv.transpose()*(-Jpinv*dJdq*Jpinv
                                                +Jpinv*Jpinv.transpose()*dJdq.transpose()*(I6-J*Jpinv)
                                                +(In-Jpinv*J)*dJdq.transpose()*Jpinv.transpose()*Jpinv);
            for (int i = 0; i < n; i++) {
                dJnorm_dq(k) += Jdjdq_(i, i);
            }
            for (int j = 0; j < 6; j++) {
                dJpinvnorm_dq(k) += Jpinvdjpinvdq_(j, j);
            }
            dJnorm_dq(k) *= 1/J.norm();
            dJpinvnorm_dq(k) *= 1/Jpinv.norm();

            Jdext(k) = -pow(dext,2)*(dJnorm_dq(k)*Jpinv.norm()+J.norm()*dJpinvnorm_dq(k));
        }
    }

    dexterity_.insert(std::make_pair(frameID, dext));

    return Jdext;
}

Eigen::MatrixXd ArmModel::DexterityJacobian(const std::string& frameID) 
{
    if (dexterityJacobians_.find(frameID) == dexterityJacobians_.end()) {
        dexterityJacobians_.insert(std::make_pair(frameID, EvaluateDexterity(frameID)));
    }
    return (dexterityJacobians_.at(frameID));
}

double ArmModel::Dexterity(const std::string& frameID) 
{
    if (dexterity_.find(frameID) == dexterity_.end()) {
        std::cerr << "nimblbot::ArmModel, Dexterity() no frame " << frameID << std::endl;
    }
    return dexterity_.at(frameID);
}
}
