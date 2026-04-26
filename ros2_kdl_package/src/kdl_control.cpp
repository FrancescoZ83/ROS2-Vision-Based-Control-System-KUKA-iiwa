#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd,
                                      KDLRobot &_robot)
{   
    KDLRobot* rob = &_robot;
    
    // read current state
    Eigen::VectorXd q = rob->getJntValues();
    Eigen::VectorXd dq = rob->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;
    Eigen::VectorXd ddqd = _ddqd.data;
    
    return rob->getJsim() * (ddqd + _Kd*de + _Kp*e) + rob->getCoriolis() + rob->getGravity();
}


Eigen::MatrixXd KDLController::computeDampedPseudoInverse(const Eigen::MatrixXd& J, double lambda) {

    Eigen::MatrixXd JtJ = J.transpose() * J;
    Eigen::MatrixXd damping = lambda * lambda * Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols());
    Eigen::MatrixXd J_damped_pinv = (JtJ + damping).inverse() * J.transpose();
    return J_damped_pinv;
}

void KDLController::CLIK(KDL::Frame &_desPos,
                         KDL::Twist &_desVel,
                         KDL::Twist &_desAcc,
                         double _Kp, double _Kd,
                         KDL::JntArray &_dq, KDL::JntArray &_dqd, KDL::JntArray &_dqdd,
                         double int_t, KDLRobot &_robot, double lam)
{
    KDLRobot* rob = &_robot;
    
    // calculate errors
    Vector6d err, derr;
    computeErrors(_desPos, rob->getEEFrame(), _desVel, rob->getEEVelocity(), err, derr);
    KDL::Twist dxdd = _desAcc;
    
    // calculate damped pseudoinverse
    Eigen::MatrixXd Jdamp = this->computeDampedPseudoInverse(rob->getEEJacobian().data, lam);
    
    // calculate joint trajectories
    _dqdd.data = Jdamp * (toEigen(dxdd) + _Kd*derr + _Kp*err - rob->getEEJacDotqDot());
    _dqd.data += _dqdd.data*int_t;
    _dq.data += _dqd.data*int_t;
}


Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp,
                                      double _Kdp, KDLRobot &_robot, double lam)
{   
    KDLRobot* rob = &_robot;
    
    // calculate errors
    Vector6d err, derr;
    computeErrors(_desPos, rob->getEEFrame(), _desVel, rob->getEEVelocity(), err, derr);
    KDL::Twist dxdd = _desAcc;
    
    // calculate damped pseudoinverse
    Eigen::MatrixXd Jdamp = this->computeDampedPseudoInverse(rob->getEEJacobian().data, lam);
    
    // calculate y and then tau
    Eigen::VectorXd y = Jdamp * (toEigen(dxdd) + _Kdp*derr + _Kpp*err - rob->getEEJacDotqDot());
    
    return rob->getJsim() * y + rob->getCoriolis() + rob->getGravity();
}

