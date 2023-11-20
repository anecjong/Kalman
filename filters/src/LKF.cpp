#include "filters/LKF.hpp"

LinearKalmanFilter::LinearKalmanFilter(
    Eigen::MatrixXd A,Eigen::MatrixXd H,Eigen::MatrixXd Q,Eigen::MatrixXd R)
    :m_matrixxd_A(A), m_matrixxd_H(H), m_matrixxd_Q(Q), m_matrixxd_R(R) {}

void LinearKalmanFilter::setInitalValue(Eigen::VectorXd x_0,Eigen::MatrixXd P_0){
    m_vectorxd_x_estimated = x_0;
    m_matrixxd_P_estimated = P_0;
    return;
}

Eigen::VectorXd LinearKalmanFilter::getEstimatedValue(){
    return m_vectorxd_x_estimated;
}

Eigen::MatrixXd LinearKalmanFilter::getCovariance(){
    return m_matrixxd_P_estimated;
}


void LinearKalmanFilter::update(Eigen::VectorXd z){
    // prediction
    Eigen::VectorXd x_predicted = m_matrixxd_A * m_vectorxd_x_estimated;
    Eigen::MatrixXd P_predicted = m_matrixxd_A * m_matrixxd_P_estimated * m_matrixxd_A.transpose() + m_matrixxd_Q;

    Eigen::MatrixXd kalman_gain = P_predicted * m_matrixxd_H.transpose() * (m_matrixxd_H * P_predicted * m_matrixxd_H.transpose() + m_matrixxd_R).inverse();

    // update
    m_vectorxd_x_estimated = x_predicted + kalman_gain * ( z - m_matrixxd_H * x_predicted);
    m_matrixxd_P_estimated = P_predicted - kalman_gain * m_matrixxd_H * P_predicted;

    return;
}