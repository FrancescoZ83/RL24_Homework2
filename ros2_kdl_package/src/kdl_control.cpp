#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::PDplus(KDL::JntArray &_qd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;

    return _Kp*e + _Kd*dq + robot_->getGravity();
}


inline Eigen::VectorXd MytoEigen(const KDL::Frame& F)
{
    Eigen::VectorXd e(6);  // Create a VectorXd of size 6

    // Convert position from KDL::Vector to Eigen::Vector3d
    Eigen::Vector3d position = toEigen(F.p);
    e.head<3>() = position;  // Assign the first 3 elements to position

    // Convert orientation from KDL::Rotation to Eigen::Matrix3d and then to Eigen::Vector3d
    Eigen::Matrix3d orientation = toEigen(F.M);
    Eigen::Quaterniond quaternion(orientation);
    Eigen::Vector3d orientationVec(quaternion.x(), quaternion.y(), quaternion.z());
    e.tail<3>() = orientationVec;  // Assign the last 3 elements to orientation

    return e;  // Return the filled Eigen::VectorXd
}

inline Eigen::VectorXd MytoEigen(const KDL::Twist& T)
{
    Eigen::VectorXd e(6);  // Create a VectorXd of size 6

    // Convert linear velocity (KDL::Vector) to Eigen::Vector3d
    Eigen::Vector3d linear = toEigen(T.vel);
    e.head<3>() = linear;  // Assign the first 3 elements to the linear velocity

    // Convert angular velocity (KDL::Vector) to Eigen::Vector3d
    Eigen::Vector3d angular = toEigen(T.rot);
    e.tail<3>() = angular;  // Assign the last 3 elements to the angular velocity

    return e;  // Return the filled Eigen::VectorXd
}



Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    Eigen::VectorXd e = MytoEigen(_desPos); - MytoEigen(robot_->getEEFrame());
    Eigen::VectorXd de = MytoEigen(_desVel) - MytoEigen(robot_->getEEVelocity());
    
    //e.tail(3).setZero();
    //de.tail(3).setZero();
    
    Eigen::VectorXd y = pseudoinverse(robot_->getEEJacobian().data) * (MytoEigen(_desAcc) + _Kdp*de + _Kpp*e - robot_->getEEJacDotqDot());
    
    return robot_->getJsim() * y + robot_->getCoriolis() + robot_->getGravity();
}

