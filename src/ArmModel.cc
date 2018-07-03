/*
 * ctrl_armModel.cc
 *
 *  Created on: Jul 25, 2015
 *      Author: francescow
 */

#ifndef CMAT_STANDALONE
#define CMAT_STANDALONE
#endif

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
#include "RobotLink.h"
#include "SVD.h"
#include "Types.h"
#include "rml_internal/Futils.h"

using std::cout;
using std::endl;

namespace rml {

ArmModel::ArmModel(std::string id) : numberOfJoints_(0), modelReadFromFile_(false), modelInitialized_(false), isMapInitialized_(false),
    id_(id), mu_(0.0)
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

void ArmModel::SetJointsPosition(const Eigen::VectorXd& q)
{

    if (!modelInitialized_) {
        std::cout << "ERROR: Called SetJointPosition() on an unitialised ArmModel().\nExiting..." << std::endl;
        exit(0);
    }
    q_ = q;

    EvaluatebTt();
    EvaluatebJt();
    EvaluatedJdqNumeric();

    if (!isMapInitialized_)
    {
        transformation_.erase(transformation_.begin(), transformation_.end());
        jacobians_.erase(jacobians_.begin(), jacobians_.end());
        transformation_.insert(std::make_pair(id_ + "_Tool", bTt_));
        jacobians_.insert(std::make_pair(id_ + "_Tool", bJt_));
        //updating the joint jacobians
        for (int i = 0; i < numberOfJoints_; i++)
        {
            transformation_.insert(std::make_pair(id_ + "_Joint_" + std::to_string(i), baseTei_.at(i)));
            jacobians_.insert(std::make_pair(id_ + "_Joint_" + std::to_string(i), EvaluateBase2JointJacobian(i)));
        }
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, IndexedTMat>::iterator iter = attachedBodyFrames_.begin();
             iter != attachedBodyFrames_.end();
             ++iter)
        {
            std::string id = iter->first;
            transformation_.insert(std::make_pair(id, GetAttachedBodyTransf(id)));
            jacobians_.insert(std::make_pair(id, GetAttachedBodyJacobian(id)));
        }
        isMapInitialized_ = true;
    }
    else
    {
        transformation_.find((id_ + "_Tool"))->second = bTt_;
        jacobians_.find((id_ + "_Tool"))->second = bJt_;
        //updating the joint jacobians
        for (int i = 0; i < numberOfJoints_; i++)
        {
            transformation_.find(id_ + "_Joint_" + std::to_string(i))->second = baseTei_.at(i);
            jacobians_.find(id_ + "_Joint_" + std::to_string(i))->second = EvaluateBase2JointJacobian(i);
        }
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, IndexedTMat>::iterator iter = attachedBodyFrames_.begin();
             iter != attachedBodyFrames_.end();
             ++iter)
        {
            std::string id = iter->first;
            transformation_.find(id)->second = GetAttachedBodyTransf(id);
            jacobians_.find(id)->second = GetAttachedBodyJacobian(id);
        }
    }
}

void ArmModel::SetJointsVelocity(const Eigen::VectorXd& qdot)
{

    if (!modelInitialized_)
    {
        std::cout << "ERROR: Called SetJointsVelocity() on an unitialised ArmModel().\nExiting..." << std::endl;
        exit(0);
    }
    q_dot_ = qdot;
}

void ArmModel::SetJointsAcceleration(const Eigen::VectorXd& qddot)
{

    if (!modelInitialized_) {
        std::cout << "ERROR: Called SetJointsAcceleration() on an unitialised ArmModel().\nExiting..." << std::endl;
        exit(0);
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

void ArmModel::EvaluatebJt()
{


    for (int jointNumber = numberOfJoints_ - 1; jointNumber >= 0; jointNumber--)
        BackwardDirectGeometryToolFrame(jointNumber);
    bJt_ = (h_[0]);
    for (int i = 1; i < numberOfJoints_; i++) {
        bJt_ = RightJuxtapose(bJt_, h_[i]);
    }
}

void ArmModel::EvaluatebTt()
{
    for (int jointNumber = 0; jointNumber < numberOfJoints_; jointNumber++) {
        ForwardDirectGeometry(jointNumber);
    }
    bTt_ = baseTei_[numberOfJoints_ - 1] * eTt_;
}

void ArmModel::ForwardDirectGeometry(int jointNumber)
{

    // wTbi is the transformation between the base of joint <jointNumber> and the world frame <w>
    if (jointNumber == 0)
    {
        baseTbi_ = baseTb0_;
    }
    else
    {
        // in this case, the base of joint <jointNumber> is the position of the end-effector of the joint <jointNumber - 1>
        baseTbi_ = baseTei_[jointNumber - 1];
    }

    // biTri is the constant transformation between the base of the joint <i> and its end-effector
    // biTei also takes into account the actual rotation of the joint, so
    // biTei = biTri * Tz(qi)

    if (links_.at(jointNumber).Type() == JointType::Revolute)
    {
        Eigen::AngleAxisd rot = Eigen::AngleAxisd(q_(jointNumber), links_.at(jointNumber).Axis());
        Tz_.SetRotMatrix(rot.toRotationMatrix());

    }
    else if (links_.at(jointNumber).Type() == JointType::Prismatic)
    {
        Eigen::Vector3d transl = q_(jointNumber) * links_.at(jointNumber).Axis();
        Tz_.SetTransl(transl);
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
    h_[jointNumber].SetSecondVect3(base_ki_.cross(bTt_.GetTransl() - baseTei_[jointNumber].GetTransl()));

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

    if (numberOfJoints_ < 6)
    {
        //TODO change svd data
        mySVD.params.lambda = 0.0;
        mySVD.params.threshold = 0.0;
        /// For defective manipulators
        //std::cout << "nrow: " << dJdq_[0].rows() << " ncol:" << dJdq_[0].cols() << std::endl;
        Jpinv_ = rml::RegularizedPseudoInverse((Eigen::MatrixXd)bJt_.transpose(), mySVD);
        //Jpinv_ = .RegPseudoInverse(, );
        for (int k = 0; k < numberOfJoints_; k++)
        {
            Jmu(k) = 0;
            djdqJpinv_ = dJdq_[k].transpose() * Jpinv_;

            for (int i = 0; i < 5; i++) // now 5 is correct
                Jmu(k) += djdqJpinv_(i, i);

            Jmu(k) *= mySVD.results.mu;
        }
        //std::cout<<"mu "<<mySVD.results.mu<<std::endl;

        //mu.PrintMtx("mu");
    }
    else
    {
        mySVD.params.lambda = 0.01;
        mySVD.params.threshold = 0.01;
        Jpinv_ = rml::RegularizedPseudoInverse(bJt_, mySVD);

        for (int k = 0; k < numberOfJoints_; k++)
        {
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
    for (int i = 0; i < numberOfJoints_ - 1; ++i)
    {
        /// Computing the single q variation vector dQ.
        dQ = ZeroQ_; // Initialising a zero vector
        dQ(i) = delta_q; // Getting the q variation
        qVar = q_orig + dQ; // Computing the new single q variation based on previous state

        /// Computing scalar delta_q (the denominator of the numerical integration).
        q_ = qVar;

        EvaluatebTt();
        EvaluatebJt();
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

    EvaluatebTt();
    EvaluatebJt();
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

void ArmModel::AddRigidBodyFrame(std::string ID, int jointIndex, Eigen::TransfMatrix TMat)
{

    IndexedTMat myMat(jointIndex, TMat);
    std::string idRigidFrame = id_ + "_Body_" + ID;
    attachedBodyFrames_.insert(std::make_pair(idRigidFrame, myMat));
    transformation_.insert(std::make_pair(idRigidFrame, GetAttachedBodyTransf(idRigidFrame)));
    jacobians_.insert(std::make_pair(idRigidFrame, GetAttachedBodyJacobian(idRigidFrame)));
}

Eigen::TransfMatrix ArmModel::GetAttachedBodyFrame(std::string& ID)
{
    return attachedBodyFrames_.at(ID).second;
}

Eigen::TransfMatrix ArmModel::GetAttachedBodyTransf(std::string& ID)
{

    int jointIndex = attachedBodyFrames_.at(ID).first;
    Eigen::TransfMatrix TMat = attachedBodyFrames_.at(ID).second;

    return transformation_.at(id_ + "_Joint_" + std::to_string(jointIndex)) * TMat;
}

Eigen::MatrixXd ArmModel::GetAttachedBodyJacobian(std::string& ID)
{

    int jointIndex = attachedBodyFrames_.at(ID).first;
    Eigen::TransfMatrix TMat = attachedBodyFrames_.at(ID).second;
    Eigen::Vector3d projectedTransl = transformation_.at(id_ + "_Joint_" + std::to_string(jointIndex)).GetRotMatrix() * TMat.GetTransl();
    return GetRigidBodyMatrix(projectedTransl) * jacobians_.at(id_ + "_Joint_" + std::to_string(jointIndex));

}

Eigen::TransfMatrix ArmModel::GetTransformationMatrix(const std::string matrixId)
{

    return transformation_.at(matrixId);
}

Eigen::MatrixXd ArmModel::GetJacobian(const std::string jacobianID)
{
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

const Eigen::TransfMatrix& ArmModel::GetBaseTransf()
{
    return baseTb0_; ;
}

void ArmModel::SetBaseTransf(const Eigen::TransfMatrix& baseTb0)
{
    baseTb0_ = baseTb0;
}

const Eigen::MatrixXd& ArmModel::GetBase2ToolJacobian() const
{
    return bJt_;
}

const Eigen::TransfMatrix& ArmModel::GetCurrentLinkTransf(int ji)
{
    return biTei_.at(ji);
}

const Eigen::TransfMatrix& ArmModel::GetBase2ToolTransf()
{
    return bTt_;
}

const Eigen::TransfMatrix& ArmModel::GeteTt() const
{
    return eTt_;
}

void ArmModel::SeteTt(const Eigen::TransfMatrix& eTt)
{
    eTt_ = eTt;
}

const std::vector<Eigen::MatrixXd>& ArmModel::GetdJdq() const
{
    return dJdq_;
}

RobotLink& ArmModel::GetLink(int jointIndex) throw(ArmModelException)
{
    if (jointIndex < links_.size())
        return links_.at(jointIndex);
    else
        throw ArmModelException();
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
    if (controlRef.rows() == numberOfJoints_)
    {
        controlRef_ = controlRef;
    }
    else
    {
        throw ArmModelException();
    }
}

double ArmModel::GetManipulability()
{
    return mu_;
}

void ArmModel::SetManipulability(double mu)
{
    mu_ = mu;
}

std::string ArmModel::GetID()
{
    return id_;
}

void ArmModel::SetID(std::string id)
{
    id_ = id;
}

}

