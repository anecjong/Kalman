#include "Eigen/Dense"

class LinearKalmanFilter {
private:
    Eigen::MatrixXd m_matrixxd_A;
    Eigen::MatrixXd m_matrixxd_H;
    Eigen::MatrixXd m_matrixxd_Q;
    Eigen::MatrixXd m_matrixxd_R;

    Eigen::VectorXd m_vectorxd_x_estimated;
    Eigen::MatrixXd m_matrixxd_P_estimated;

public:
    LinearKalmanFilter(
        Eigen::MatrixXd A,
        Eigen::MatrixXd H,
        Eigen::MatrixXd Q,
        Eigen::MatrixXd R
    );

    void setInitalValue(
        Eigen::VectorXd x_0,
        Eigen::MatrixXd P_0
    );

    Eigen::VectorXd getEstimatedValue();
    Eigen::MatrixXd getCovariance();

    void update(Eigen::VectorXd z);
};