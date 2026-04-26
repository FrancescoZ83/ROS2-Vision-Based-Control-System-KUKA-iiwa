#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd,
                           KDLRobot &_robot);
                           
    Eigen::MatrixXd computeDampedPseudoInverse(const Eigen::MatrixXd& J, double lambda);
    
    void CLIK(KDL::Frame &_desPos,
              KDL::Twist &_desVel,
              KDL::Twist &_desAcc,
              double _Kp, double _Kd,
              KDL::JntArray &_dq, KDL::JntArray &_dqd, KDL::JntArray &_dqdd,
              double int_t, KDLRobot &_robot, double lam);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kdp,
                           KDLRobot &_robot, double lam);

private:

    KDLRobot* robot_;

};

#endif
