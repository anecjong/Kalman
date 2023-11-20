#include <stdint.h>
#include <deque>

class BaseFilter {
protected:
    double m_estimated_value;

public:
    virtual void update(double val) = 0;
    double getEstimatedValue();
};

class AverageFilter : public BaseFilter {
private:
    uint32_t m_uint32_count = 0;

public:
    AverageFilter();
    virtual void update(double val) override;
};

class MovingAverageFilter : public BaseFilter{
private:
    uint32_t m_uint32t_avg_count;
    std::deque<double> m_deque_data;

public:
    MovingAverageFilter(uint32_t data_count);
    virtual void update(double val) override;
};

class LowPassFilter : public BaseFilter{
private:
    double m_double_alpha = 0;
    bool m_bool_initialized = false;

public:
    LowPassFilter(double alpha);
    virtual void update(double val) override;
};

class SimpleKalman : public BaseFilter{
private:
    double A;
    double H;
    double Q;
    double R;

    double x_estimated;
    double P_estimated;
    double kalman_gain;

public:
    SimpleKalman(double A, double H, double Q, double R);
    virtual void update(double z) override;

    void setInitialValue(double x_0_, double P_0_);
    double getCovariance();
    double getKalmanGain();
};