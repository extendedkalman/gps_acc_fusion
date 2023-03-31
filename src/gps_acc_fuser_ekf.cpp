// ekf.hpp
#pragma once
#include <Eigen/Dense>

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter()
  {
    x_ = Eigen::VectorXd(6);
    P_ = Eigen::MatrixXd(6, 6);
    Q_ = Eigen::MatrixXd(6, 6);
    F_ = Eigen::MatrixXd(6, 6);
    B_ = Eigen::MatrixXd(6, 6);
    H_gps_ = Eigen::MatrixXd(3, 6);
    H_imu_ = Eigen::MatrixXd(3, 6);
    Hj_gps_ = Eigen::MatrixXd(3, 6);
    Hj_imu_ = Eigen::MatrixXd(3, 6);
  }

  void initialize(const Eigen::VectorXd& x, const Eigen::MatrixXd& P)
  {
    x_ = x;
    P_ = P;
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 1e-6;
    F_ = Eigen::MatrixXd::Identity(6, 6);
    B_ = Eigen::MatrixXd::Identity(6, 6);

    H_gps_.setZero();
    H_gps_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);

    H_imu_.setZero();
    H_imu_.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
  }
  void predict(const Eigen::VectorXd& u, double dt)
  {
    F_.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;
    x_ = F_ * x_ + B_ * u;
    P_ = F_ * P_ * F_.transpose() + Q_;
  }
  void update_gps(const Eigen::VectorXd& z_gps, const Eigen::MatrixXd& R_gps)
  {
    linearize_gps();
    Eigen::MatrixXd K = P_ * Hj_gps_.transpose() * (Hj_gps_ * P_ * Hj_gps_.transpose() + R_gps).inverse();
    x_ = x_ + K * (z_gps - H_gps_ * x_);
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * Hj_gps_) * P_;
  }
  void update_imu(const Eigen::VectorXd& z_imu, const Eigen::MatrixXd& R_imu)
  {
    linearize_imu();
    Eigen::MatrixXd K = P_ * Hj_imu_.transpose() * (Hj_imu_ * P_ * Hj_imu_.transpose() + R_imu).inverse();
    x_ = x_ + K * (z_imu - H_imu_ * x_);
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * Hj_imu_) * P_;
  }

  Eigen::VectorXd get_state() const { return x_; }
  Eigen::MatrixXd get_covariance() const { return P_; }

private:
  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // Process noise covariance matrix
  Eigen::MatrixXd Q_;

  // System state transition model
  Eigen::MatrixXd F_;

  // Discrete-time approximation of the system state transition model
  Eigen::MatrixXd Ad_;

  // System input matrix
  Eigen::MatrixXd B_;

  // Discrete-time approximation of the system input matrix
  Eigen::MatrixXd Bd_;

  // Observation model for GPS
  Eigen::MatrixXd H_gps_;

  // Observation model for IMU
  Eigen::MatrixXd H_imu_;

  // Linearization of observation models
  Eigen::MatrixXd Hj_gps_, Hj_imu_;

  void linearize_gps()
  {
    Hj_gps_ = H_gps_;
  }
  void linearize_imu()
  {
    Hj_imu_ = H_imu_;
  }
};


