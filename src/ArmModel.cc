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

ArmModel::ArmModel(std::string id)
    : modelInitialized_(false)
    , isMapInitialized_(false)
    , numberOfJoints_(0)
    , modelReadFromFile_(false)
    , id_(id)
    , mu_(0.0)
{
}

ArmModel::~ArmModel()
{
}

void ArmModel::AddLink(JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double joinLimMax)
{

    links_.push_back(RobotLink(type, axis, baseTransf, jointLimMin, joinLimMax));
    numberOfJoints_ = links_.size();

    //	cout << numberOfJoints_ << " - ";
    //	futils::PrettyPrint(links_.back().baseTransf_, "baseTransf");
    baseTei_.push_back(Eigen::TransfMatrix());
    biTei_.push_back(Eigen::TransfMatrix());
    h_.push_back(Eigen::Vector6d());
    dJdq_.push_back(Eigen::MatrixXd());
    for (auto&& i : dJdq_) // access by forwarding reference, the type of i is auto&
        i.resize(6, numberOfJoints_);

    bJt_.resize(6, numberOfJoints_);
    ZeroQ_ = Eigen::VectorXd::Zero(numberOfJoints_);
    q_ = ZeroQ_;
    q_dot_ = ZeroQ_;
    q_ddot_ = ZeroQ_;
    controlRef_ = ZeroQ_;

    modelInitialized_ = true;
    isMapInitialized_ = false;
    SetJointsPosition(ZeroQ_);
}

void ArmModel::SetJointsPosition(const Eigen::VectorXd& q) throw(std::exception)
{

    if (!modelInitialized_) {
        ArmModelNotInitializedException armModelNotIntialized;
        armModelNotIntialized.SetID("SetJointPosition");
        throw(armModelNotIntialized);
    }

    if (q_.size() != q.size()) {
        ArmModelWrongJointSizeException armModelWrongJointSize;
        armModelWrongJointSize.SetID("SetJointPosition");
        throw(armModelWrongJointSize);
    }
    q_ = q;

    //EvaluatebTt();
    //EvaluatebJt();
    EvaluateTotalForwardGeometry();
    EvaluateBase2JointJacobian(numberOfJoints_ - 1);
    EvaluatedJdqNumeric();

    if (!isMapInitialized_) {
        transformation_.erase(transformation_.begin(), transformation_.end());
        jacobians_.erase(jacobians_.begin(), jacobians_.end());
        //transformation_.insert(std::make_pair(id_ + FrameID::Tool, bTt_));
        jacobians_.insert(std::make_pair(id_ + FrameID::Tool, bJt_));
        //updating the joint jacobians
        for (int i = 0; i < numberOfJoints_; i++) {
            transformation_.insert(std::make_pair(id_ + FrameID::Joint + std::to_string(i), baseTei_.at(i)));
            jacobians_.insert(std::make_pair(id_ + FrameID::Joint + std::to_string(i), EvaluateBase2JointJacobian(i)));
        }
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, IndexedTMat>::iterator iter = rigidBodyFrames_.begin();
             iter != rigidBodyFrames_.end();
             ++iter) {
            std::string id = iter->first;
            transformation_.insert(std::make_pair(id, GetRigidBodyTransf(id)));
            jacobians_.insert(std::make_pair(id, GetRigidBodyJacobian(id)));
        }
        isMapInitialized_ = true;
    } else {
        //transformation_.find((id_ + FrameID::Tool))->second = bTt_;
        jacobians_.find((id_ + FrameID::Tool))->second = bJt_;
        //updating the joint jacobians
        for (int i = 0; i < numberOfJoints_; i++) {
            transformation_.find(id_ + FrameID::Joint + std::to_string(i))->second = baseTei_.at(i);
            jacobians_.find(id_ + FrameID::Joint + std::to_string(i))->second = EvaluateBase2JointJacobian(i);
        }
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, IndexedTMat>::iterator iter = rigidBodyFrames_.begin();
             iter != rigidBodyFrames_.end();
             ++iter) {
            std::string id = iter->first;
            transformation_.find(id)->second = GetRigidBodyTransf(id);
            jacobians_.find(id)->second = GetRigidBodyJacobian(id);
        }
    }
}

void ArmModel::SetJointsVelocity(const Eigen::VectorXd& qdot) throw(std::exception)
{

    if (!modelInitialized_) {
        ArmModelNotInitializedException armModelNotIntialized;
        armModelNotIntialized.SetID("SetJointVelocity");
        throw(armModelNotIntialized);
    }
    if (qdot.size() != q_dot_.size()) {
        ArmModelWrongJointSizeException armModelWrongJointSize;
        armModelWrongJointSize.SetID("SetJointVelocity");
        throw(armModelWrongJointSize);
    }
    q_dot_ = qdot;
}

void ArmModel::SetJointsAcceleration(const Eigen::VectorXd& qddot) throw(std::exception)
{

    if (!modelInitialized_) {
        ArmModelNotInitializedException armModelNotIntialized;
        armModelNotIntialized.SetID("SetJointAcceleration");
        throw(armModelNotIntialized);
    }
    if (qddot.size() != q_ddot_.size()) {
        ArmModelWrongJointSizeException armModelWrongJointSize;
        armModelWrongJointSize.SetID("SetJointAcceleration");
        throw(armModelWrongJointSize);
    }
    q_ddot_ = qddot;
}

const Eigen::VectorXd& ArmModel::GetJointsPosition() const
{
    return q_;
}

const Eigen::VectorXd& ArmModel::GetJointsVelocity() const
{
    return q_dot_;
}

const Eigen::VectorXd& ArmModel::GetJointsAcceleration() const
{
    return q_ddot_;
}

/*void ArmModel::EvaluatebJt()
{


    for (int jointNumber = numberOfJoints_ - 1; jointNumber >= 0; jointNumber--)
        BackwardDirectGeometryToolFrame(jointNumber);
    bJt_ = (h_[0]);
    for (int i = 1; i < numberOfJoints_; i++) {
        bJt_ = RightJuxtapose(bJt_, h_[i]);
    }
}*/

void ArmModel::EvaluateTotalForwardGeometry()
{
    for (int jointNumber = 0; jointNumber < numberOfJoints_; jointNumber++) {
        ForwardDirectGeometry(jointNumber);
    }
}

void ArmModel::ForwardDirectGeometry(int jointNumber)
{

    // wTbi is the transformation between the base of joint <jointNumber> and the world frame <w>
    /*if (jointNumber == 0)
    {
        baseTbi_ = baseTb0_;
    }
    else
    {*/
    // in this case, the base of joint <jointNumber> is the position of the end-effector of the joint <jointNumber - 1>
    baseTbi_ = baseTei_[jointNumber - 1];
    //}

    // biTri is the constant transformation between the base of the joint <i> and its end-effector
    // biTei also takes into account the actual rotation of the joint, so
    // biTei = biTri * Tz(qi)

    if (links_.at(jointNumber).Type() == JointType::Revolute) {
        Eigen::AngleAxisd rot = Eigen::AngleAxisd(q_(jointNumber), links_.at(jointNumber).Axis());
        Tz_.SetRotMatrix(rot.toRotationMatrix());
    } else if (links_.at(jointNumber).Type() == JointType::Prismatic) {
        Eigen::Vector3d transl = q_(jointNumber) * links_.at(jointNumber).Axis();
        Tz_.SetTransl(transl);
    } else if (links_.at(jointNumber).Type() == JointType::Fixed) {
        Tz_.setIdentity();
    }

    // biTei = biTri * Tz(qi)

    biTei_[jointNumber] = links_.at(jointNumber).BaseTransf() * Tz_;

    // wTei is the transformation between the end-effector of joint <i> and world frame <w>
    // wTei = wTbi * biTei
    baseTei_[jointNumber] = baseTbi_ * biTei_[jointNumber];
    //cout << "forward index = " << jointNumber << endl;
    //biTri_[jointNumber - 1].PrintMtx("biTri");
    //Tz_.PrintMtx("Tz");
    //wTei_[jointNumber - 1].PrintMtx("wTei");
}

void ArmModel::BackwardDirectGeometry(int jointNumber, int endEffectorIndex)
{
    // Calcolo w_ki
    // Dal momento che ri_ki e ei_ki sono ruotate lungo ki e per convenzione
    // ogni giunto ruota lungo z si ha che ri_ki = ei_ki = [ 0 0 1 ]'
    // Di conseguenza w_ki e' la 3a colonna della R che lo proietta sul mondo
    base_ki_ = baseTei_.at(jointNumber).block(0, 2, 3, 1); //GetSubMatrix(1, 3, 3, 3));

    h_[jointNumber].SetFirstVect3(base_ki_);
    h_[jointNumber].SetSecondVect3(base_ki_.cross(baseTei_[endEffectorIndex].GetTransl() - baseTei_[jointNumber].GetTransl()));
}

void ArmModel::BackwardDirectGeometryToolFrame(int jointNumber)
{
    // Calcolo w_ki
    // Dal momento che ri_ki e ei_ki sono ruotate lungo ki e per convenzione
    // ogni giunto ruota lungo z si ha che ri_ki = ei_ki = [ 0 0 1 ]'
    // Di conseguenza w_ki e' la 3a colonna della R che lo proietta sul mondo
    base_ki_ = baseTei_.at(jointNumber).block(0, 2, 3, 1); //GetSubMatrix(1, 3, 3, 3));

    h_[jointNumber].SetFirstVect3(base_ki_);
    h_[jointNumber].SetSecondVect3(base_ki_.cross(baseTei_[numberOfJoints_ - 1].GetTransl() - baseTei_[jointNumber].GetTransl()));

    //cout << "backward index = " << jointNumber << endl;
    //wTei_[jointNumber - 1].PrintMtx("wTei");
    //w_ki_.PrintMtx("wki");
    //h_[jointNumber - 1].PrintMtx("h");
}

void ArmModel::EvaluateManipulability(Eigen::MatrixXd& Jmu)
{

    Jmu.resize(1, numberOfJoints_);
    Jmu.setZero();

    RegularizationData mySVD;

    if (numberOfJoints_ < 6) {
        //TODO change svd data
        mySVD.params.lambda = 0.0;
        mySVD.params.threshold = 0.00;
        /// For defective manipulators
        //std::cout << "nrow: " << dJdq_[0].rows() << " ncol:" << dJdq_[0].cols() << std::endl;
        Jpinv_ = rml::RegularizedPseudoInverse((Eigen::MatrixXd)bJt_.transpose(), mySVD);
        //Jpinv_ = .RegPseudoInverse(, );
        for (int k = 0; k < numberOfJoints_; k++) {
            Jmu(k) = 0;
            djdqJpinv_ = dJdq_[k].transpose() * Jpinv_;

            for (int i = 0; i < numberOfJoints_; i++)
                Jmu(k) += djdqJpinv_(i, i);

            Jmu(k) *= mySVD.results.mu;
        }
        //std::cout<<"mu "<<mySVD.results.mu<<std::endl;

        //mu.PrintMtx("mu");
    } else {
        mySVD.params.lambda = 0.01;
        mySVD.params.threshold = 0.01;
        Jpinv_ = rml::RegularizedPseudoInverse(bJt_, mySVD);

        for (int k = 0; k < numberOfJoints_; k++) {
            Jmu(k) = 0;
            djdqJpinv_ = dJdq_[k] * Jpinv_;
            for (int i = 0; i < 6; i++)
                Jmu(k) += djdqJpinv_(i, i);
            Jmu(k) *= mySVD.results.mu;
        }
    }

    mu_ = mySVD.results.mu;
}

void ArmModel::EvaluatedJdqNumeric()
{

    Eigen::MatrixXd bJt_0, bJt_dQ;
    Eigen::MatrixXd dQ, qVar, q_orig;
    double delta_q = 1E-6;
    q_orig = q_;
    bJt_0 = GetBase2ToolJacobian();
    //futils::PrettyPrint(wJt_0,"wJt_0");

    /// Here we iterate till "numJoints - 1" since the last joint will not influence
    /// on the Jacobian since nothing is connected to it.
    for (int i = 0; i < numberOfJoints_ - 1; ++i) {
        /// Computing the single q variation vector dQ.
        dQ = ZeroQ_; // Initialising a zero vector
        dQ(i) = delta_q; // Getting the q variation
        qVar = q_orig + dQ; // Computing the new single q variation based on previous state

        /// Computing scalar delta_q (the denominator of the numerical integration).
        q_ = qVar;

        EvaluateTotalForwardGeometry();
        EvaluateBase2JointJacobian(numberOfJoints_ - 1);
        bJt_dQ = GetBase2ToolJacobian();
        //futils::PrettyPrint(wJt_dQ,"wJt_dQ");
        for (int iJrow = 0; iJrow < 6; ++iJrow) {
            for (int iJcol = 0; iJcol < numberOfJoints_; ++iJcol) {
                //std::cout << "(i,j) = " << iJrow << "," << iJcol << std::endl;
                //futils::PrettyPrint(dJdq_[i],"dJdq[i]");
                dJdq_.at(i)(iJrow, iJcol) = (bJt_dQ(iJrow, iJcol) - bJt_0(iJrow, iJcol)) / delta_q;
                if (std::fabs(dJdq_.at(i)(iJrow, iJcol)) < 1E-6)
                    dJdq_.at(i)(iJrow, iJcol) = 0.0;
            }
        }
    }
    q_ = q_orig;

    EvaluateTotalForwardGeometry();
    EvaluateBase2JointJacobian(numberOfJoints_ - 1);
}

Eigen::TransfMatrix ArmModel::GetBase2JointTransf(int jointIndex)
{
    return baseTei_[jointIndex];
}

Eigen::MatrixXd ArmModel::EvaluateBase2JointJacobian(int jointIndex)
{
    //Here we calculate the h columns involved in our joint's jacobian
    for (int jointNumber = jointIndex; jointNumber >= 0; jointNumber--)
        BackwardDirectGeometry(jointNumber, jointIndex);

    //Then we set all the remainings columns to zero
    for (int i = jointIndex + 1; i < numberOfJoints_; i++) {
        h_[i].setZero();
    }

    Eigen::MatrixXd bJj; // = (h_[0]);
    for (int i = 0; i < numberOfJoints_; i++) {
        bJj = RightJuxtapose(bJj, h_[i]);
    }
    return bJj;
}

void ArmModel::SetRigidBodyFrame(std::string ID, int jointIndex, Eigen::TransfMatrix TMat) throw(std::exception)
{
    if (jointIndex >= numberOfJoints_) {
        ArmModelNotExistingJointException armModelNotExistingJoint;
        armModelNotExistingJoint.SetID("AddRigidBodyFrame");
        throw(armModelNotExistingJoint);
    }
    std::string idRigidFrame = id_ + FrameID::Body + ID;

    // Check if rigid body is already present
    if (rigidBodyFrames_.find(idRigidFrame) != rigidBodyFrames_.end()) {
        // Check if the associated joint is the same otherwise throw exception
        if (rigidBodyFrames_.at(idRigidFrame).first == jointIndex) {
            rigidBodyFrames_.at(idRigidFrame).second = TMat;
            transformation_.at(idRigidFrame) = GetRigidBodyTransf(idRigidFrame);
            jacobians_.at(idRigidFrame) = GetRigidBodyJacobian(idRigidFrame);
        } else {
            LabelAlreadyUsedException except;
            except.SetID("SetRigidBodyFrame");
            throw(except);
        }
    } else {
        IndexedTMat myMat(jointIndex, TMat);
        rigidBodyFrames_.insert(std::make_pair(idRigidFrame, myMat));
        transformation_.insert(std::make_pair(idRigidFrame, GetRigidBodyTransf(idRigidFrame)));
        jacobians_.insert(std::make_pair(idRigidFrame, GetRigidBodyJacobian(idRigidFrame)));
    }
}

Eigen::TransfMatrix ArmModel::GetRigidBodyFrame(std::string& ID) throw(std::exception)
{
    if (rigidBodyFrames_.find(ID) == rigidBodyFrames_.end()) {
        ArmModelWrongLabelException armModelWrongLabel;
        armModelWrongLabel.SetID("GetAttachedBodyFrame");
        throw(armModelWrongLabel);
    }
    return rigidBodyFrames_.at(ID).second;
}

Eigen::TransfMatrix ArmModel::GetRigidBodyTransf(std::string& ID)
{

    int jointIndex = rigidBodyFrames_.at(ID).first;
    Eigen::TransfMatrix TMat = rigidBodyFrames_.at(ID).second;

    return transformation_.at(id_ + FrameID::Joint + std::to_string(jointIndex)) * TMat;
}

Eigen::MatrixXd ArmModel::GetRigidBodyJacobian(std::string& ID)
{

    int jointIndex = rigidBodyFrames_.at(ID).first;
    Eigen::TransfMatrix TMat = rigidBodyFrames_.at(ID).second;
    Eigen::Vector3d projectedTransl = transformation_.at(id_ + FrameID::Joint + std::to_string(jointIndex)).GetRotMatrix() * TMat.GetTransl();
    return GetRigidBodyMatrix(projectedTransl) * jacobians_.at(id_ + FrameID::Joint + std::to_string(jointIndex));
}

Eigen::TransfMatrix ArmModel::GetTransformationMatrix(const std::string matrixId) throw(std::exception)
{
    //// TODO: GIVE INFO ON THE FACT THE INDEX IS OUT OF RANGE IF GREATER THAN NUMJOINTS
    if (transformation_.find(matrixId) == transformation_.end()) {
        ArmModelWrongLabelException armModelWrongLabel;
        armModelWrongLabel.SetID("GetTransformationMatrix");
        throw(armModelWrongLabel);
    }
    return transformation_.at(matrixId);
}

Eigen::MatrixXd ArmModel::GetJacobian(const std::string jacobianID) throw(std::exception)
{
    if (jacobians_.find(jacobianID) == jacobians_.end()) {
        ArmModelWrongLabelException armModelWrongLabel;
        armModelWrongLabel.SetID("GetJacobian");
        throw(armModelWrongLabel);
    }
    return jacobians_.at(jacobianID);
}

/*void ArmModel::ReadModelMatricesFromFile(std::string folder_path) {

	modelReadFromFile_ = true;
	modelInitialized_ = true;
	std::cout << "[ReadModelMatricesFromFile] Not implemented yet" << std::endl;
	exit(0);

}*/

int ArmModel::GetNumJoints() const
{
    return links_.size();
}

/*const Eigen::TransfMatrix& ArmModel::GetBaseTransf()
{
    return baseTb0_; ;
}*/

/*void ArmModel::SetBaseTransf(const Eigen::TransfMatrix& baseTb0)
{
    baseTb0_ = baseTb0;
}*/

const Eigen::MatrixXd& ArmModel::GetBase2ToolJacobian() const
{
    return bJt_;
}

const Eigen::TransfMatrix& ArmModel::GetCurrentLinkTransf(int ji)
{
    return biTei_.at(ji);
}

const std::vector<Eigen::MatrixXd>& ArmModel::GetdJdq() const
{
    return dJdq_;
}

RobotLink& ArmModel::GetLink(int jointIndex) throw(std::exception)
{
    if (jointIndex < links_.size())
        return links_.at(jointIndex);
    else {
        ArmModelNotExistingJointException armModelNotExistingJoint;
        armModelNotExistingJoint.SetID("GetLink");
        throw(armModelNotExistingJoint);
    }
}

bool ArmModel::IsModelInitialized() const
{
    return modelInitialized_;
}

const Eigen::VectorXd& ArmModel::GetControlVector() const
{
    return controlRef_;
}

void ArmModel::SetControlVector(const Eigen::VectorXd& controlRef) throw(std::exception)
{
    if (controlRef.rows() == numberOfJoints_) {
        controlRef_ = controlRef;
    } else {
        ArmModelWrongJointSizeException armModelWrongJointSize;
        armModelWrongJointSize.SetID("SetControlVector");
        throw(armModelWrongJointSize);
    }
}

double ArmModel::GetManipulability()
{
    return mu_;
}

/*void ArmModel::SetManipulability(double mu)
{
    mu_ = mu;
}*/

std::string ArmModel::GetID()
{
    return id_;
}

void ArmModel::SetID(std::string id)
{
    id_ = id;
}
}
