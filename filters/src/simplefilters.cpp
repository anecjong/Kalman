#include <stdint.h>
#include <deque>
#include <filters/simplefilters.hpp>

double BaseFilter::getEstimatedValue(){
    return m_estimated_value;
}

AverageFilter::AverageFilter(): BaseFilter(){}

void AverageFilter::update(double val){
    ++m_uint32_count;
    double alpha = (m_uint32_count - 1.0) / m_uint32_count;
    m_estimated_value = alpha * m_estimated_value + (1 - alpha) * val;
}

MovingAverageFilter::MovingAverageFilter(uint32_t data_count): BaseFilter(), m_uint32t_avg_count(data_count) {}

void MovingAverageFilter::update(double val){
    if (m_deque_data.size() == m_uint32t_avg_count){
        m_deque_data.pop_front();
    }
    m_deque_data.push_back(val);

    m_estimated_value = 0;
    for (const auto& datum : m_deque_data){
        m_estimated_value += datum;
    }
    m_estimated_value /= m_deque_data.size();
}


LowPassFilter::LowPassFilter(double alpha){
    if (alpha > 1) alpha = 1;
    if (alpha < 0) alpha = 0;
    m_double_alpha = alpha;
}

void LowPassFilter::update(double val) {
    if (!m_bool_initialized) {
        m_estimated_value = val;
        m_bool_initialized = true;
    }
    else m_estimated_value = m_double_alpha * m_estimated_value + (1 - m_double_alpha) * val;
};

SimpleKalman::SimpleKalman(double A, double H, double Q, double R): A(A), H(H), Q(Q), R(R) {}

void SimpleKalman::setInitialValue(double x_0_, double P_0_){
    x_estimated = x_0_;
    P_estimated = P_0_;

    m_estimated_value = x_estimated;
}

void SimpleKalman::update(double z){
    double x_predict = A * x_estimated;
    double P_predict = A * P_estimated * A + Q;
    kalman_gain = P_predict * H / ( H * P_predict * H + R);

    x_estimated = x_predict + kalman_gain * (z -H * x_predict);
    P_estimated = P_predict - kalman_gain * H * P_predict;

    m_estimated_value = x_estimated;
}

double SimpleKalman::getCovariance(){
    return P_estimated;
}

double SimpleKalman::getKalmanGain(){
    return kalman_gain;
}