#include <iostream>
#include <random>
#include <vector>
#include <deque>

#include "Eigen/Dense"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include "filters/simplefilters.hpp"
#include "filters/LKF.hpp"

void noise_removal();
void get_speed_from_position();
void get_position_from_speed();

int main(int argc, char**argv){
    // noise_removal();
    // get_speed_from_position();
    get_position_from_speed();
    return 0;
}

void noise_removal(){
    const double voltage_mean = 14;

    // filters
    AverageFilter avg_filter;
    MovingAverageFilter moving_avg_filter(5);
    LowPassFilter low_pass_filter(0.3);
    SimpleKalman kalman_filter(1, 1, 0, 4);
    kalman_filter.setInitialValue(voltage_mean, 6);

    const double mean = 0.0;
    const double stddev = 5;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    double counts = 50;
    std::vector<double> counters,
        obesrvations,
        avg_filter_values,
        moving_avg_filter_values,
        low_pass_filter_values,
        kalman_filter_value,
        kalman_filter_covariance,
        kalman_filter_gain;

    for (int i = 0; i < counts; ++i) {
        double observation = voltage_mean + dist(generator);

        counters.push_back(i);
        obesrvations.push_back(observation);

        kalman_filter.update(observation);
        avg_filter.update(observation);
        moving_avg_filter.update(observation);
        low_pass_filter.update(observation);

        avg_filter_values.push_back(avg_filter.getEstimatedValue());
        moving_avg_filter_values.push_back(moving_avg_filter.getEstimatedValue());
        low_pass_filter_values.push_back(low_pass_filter.getEstimatedValue());
        kalman_filter_value.push_back(kalman_filter.getEstimatedValue());
        kalman_filter_covariance.push_back(kalman_filter.getCovariance());
        kalman_filter_gain.push_back(kalman_filter.getKalmanGain());
    }

    plt::figure_size(1200, 780);
    plt::title("Voltage Noise Filtering");
    plt::named_plot("Average Filter", counters, avg_filter_values);
    plt::named_plot("Moving Average Filter", counters, moving_avg_filter_values);
    plt::named_plot("Low Pass Filter", counters, low_pass_filter_values);
    plt::named_plot("Kalman Estimated Value", counters, kalman_filter_value, "--k");
    plt::named_plot("Observation", counters, obesrvations, "or");
    plt::legend();
    plt::show();

    plt::figure_size(1200, 700);
    plt::title("Covariance");
    plt::named_plot("Kalman Filter Covariance", counters, kalman_filter_covariance);
    plt::legend();
    plt::show();

    plt::figure_size(1200, 700);
    plt::title("Kalman Gain");
    plt::named_plot("Kalman Gain", counters, kalman_filter_covariance);
    plt::legend();
    plt::show();
}

void get_speed_from_position(){
    Eigen::MatrixXd A(2, 2);
    Eigen::MatrixXd H(1, 2);
    Eigen::MatrixXd Q(2, 2);
    Eigen::MatrixXd R(1, 1);

    Eigen::VectorXd x_0(2);
    Eigen::MatrixXd P_0(2, 2);

    double dt = 0.1;

    A << 1, dt,
        0, 1;
    H << 1, 0;
    Q << 1, 0,
        0, 3;
    R << 10;

    x_0 << 0,
            20;
    P_0 << 5, 0,
            0, 5;

    LinearKalmanFilter kalman_filter(A, H, Q, R);
    kalman_filter.setInitalValue(x_0, P_0);
    
    const double mean = 0.0;
    const double stddev = 1;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    double counts = 50;
    std::vector<double> counters;
    std::vector<Eigen::VectorXd> obesrvations, kalman_filter_value;

    double velocity_mean = 80.0 * 1000 / 3600; // [m/s]

    for (int i = 0; i < counts; ++i) {
        Eigen::VectorXd observation(1);
        observation << velocity_mean * (i * dt) + dist(generator);

        counters.push_back(i);
        obesrvations.push_back(observation);

        kalman_filter.update(observation);
        kalman_filter_value.push_back(kalman_filter.getEstimatedValue());
    }

    std::vector<double> positions_observed, positions_estimated;
    for (const Eigen::VectorXd observation : obesrvations){
        positions_observed.push_back(observation(0));
    }
    for (const Eigen::VectorXd estimation : kalman_filter_value){
        positions_estimated.push_back(estimation(0));
    }

    plt::figure_size(1200, 780);
    plt::title("Position");
    plt::named_plot("Kalman Estimated Value", counters, positions_estimated, "--k");
    plt::named_plot("Observation", counters, positions_observed, "or");
    plt::legend();
    plt::show();

    std::vector<double> speed_estimated, speed_80(counts, 80);
    for (const Eigen::VectorXd estimation : kalman_filter_value){
        speed_estimated.push_back(estimation(1) * 3600.0 / 1000);
    }

    plt::figure_size(1200, 780);
    plt::title("Velocity");
    plt::named_plot("Kalman Estimated Value", counters, speed_estimated, "--k");
    plt::named_plot("Reference 80 km", counters, speed_80, "r");
    plt::legend();
    plt::ylim(0, 110);
    plt::show();
}

void get_position_from_speed(){
    Eigen::MatrixXd A(2, 2);
    Eigen::MatrixXd H(1, 2);
    Eigen::MatrixXd Q(2, 2);
    Eigen::MatrixXd R(1, 1);

    Eigen::VectorXd x_0(2);
    Eigen::MatrixXd P_0(2, 2);

    double dt = 0.1;

    A << 1, dt,
        0, 1;
    H << 0, 1;
    Q << 1, 0,
        0, 3;
    R << 10;

    x_0 << 0,
            20;
    P_0 << 5, 0,
            0, 5;

    LinearKalmanFilter kalman_filter(A, H, Q, R);
    kalman_filter.setInitalValue(x_0, P_0);
    
    const double mean = 0.0;
    const double stddev = 1;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    double counts = 50;
    std::vector<double> counters;
    std::vector<Eigen::VectorXd> obesrvations, kalman_filter_value;

    double velocity_mean = 80.0;

    for (int i = 0; i < counts; ++i) {
        Eigen::VectorXd observation(1);
        observation << velocity_mean + dist(generator);

        counters.push_back(i);
        obesrvations.push_back(observation);

        kalman_filter.update(observation);
        kalman_filter_value.push_back(kalman_filter.getEstimatedValue());
    }

    std::vector<double> speed_observed, speed_estimated;
    for (const Eigen::VectorXd observation : obesrvations){
        speed_observed.push_back(observation(0));
    }
    for (const Eigen::VectorXd estimation : kalman_filter_value){
        speed_estimated.push_back(estimation(1));
    }

    plt::figure_size(1200, 780);
    plt::title("Speed");
    plt::named_plot("Kalman Estimated Value", counters, speed_estimated, "--k");
    plt::named_plot("Observation", counters, speed_observed, "or");
    plt::legend();
    plt::show();

    std::vector<double> position_estimated, position_80;
    for (const Eigen::VectorXd estimation : kalman_filter_value){
        position_estimated.push_back(estimation(0) * 1000.0 / 3600);
    }
    for (int i = 0; i < counts; ++i) {
        position_80.push_back(velocity_mean * 1000.0 / 3600 * i * dt);
    }

    plt::figure_size(1200, 780);
    plt::title("Position");
    plt::named_plot("Kalman Estimated Value", counters, position_estimated, "--k");
    plt::named_plot("Reference 80km", counters, position_80, "r");
    plt::ylim(0, 110);
    plt::legend();
    plt::show();
}