#include <gps_acc_fusion/ekf.hpp>

ExtendedKalmanFilter::ExtendedKalmanFilter()
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

void ExtendedKalmanFilter::initialize(const Eigen::VectorXd& x, const Eigen::MatrixXd& P)
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

void ExtendedKalmanFilter::predict_constant_velo_case(const Eigen::VectorXd& u, double dt)
{
    // Apply the motion model to the state
    Eigen::VectorXd x_pred = x_;
    x_pred(0) += u(0) * dt; // x += vx * dt
    x_pred(1) += u(1) * dt; // y += vy * dt
    x_pred(2) += u(2) * dt; // z += vz * dt
    x_pred(3) += u(3) * dt; // roll += wx * dt
    x_pred(4) += u(4) * dt; // pitch += wy * dt
    x_pred(5) += u(5) * dt; // yaw += wz * dt

    // Calculate the Jacobian matrix F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(3, 0) = dt;
    F(4, 1) = dt;
    F(5, 2) = dt;

    // Update the state and covariance
    x_ = x_pred;
    P_ = F * P_ * F.transpose() + Q_;
}

void ExtendedKalmanFilter::predict(const Eigen::VectorXd& u, double dt) {
  // Define the motion model function
  auto motion_model = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
    Eigen::VectorXd x_dot(6);
    x_dot << x(3) + u(0) * dt, x(4) + u(1) * dt, x(5) + u(2) * dt, u(0), u(1), u(2);
    return x_dot;
  };

  // Apply the motion model using the Euler integration method
  x_ = x_ + motion_model(x_, u) * dt;

  // Approximate the Jacobian matrix F
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  // Update the covariance
  P_ = F * P_ * F.transpose() + Q_;
}

void ExtendedKalmanFilter::update_gps(const Eigen::VectorXd& z_gps, const Eigen::MatrixXd& R_gps)
{
    linearize_gps();
    Eigen::MatrixXd K = P_ * Hj_gps_.transpose() * (Hj_gps_ * P_ * Hj_gps_.transpose() + R_gps).inverse();
    x_ = x_ + K * (z_gps - H_gps_ * x_);
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * Hj_gps_) * P_;

}

void ExtendedKalmanFilter::update_imu(const Eigen::VectorXd& z_imu, const Eigen::MatrixXd& R_imu)
{
    linearize_imu();
    Eigen::MatrixXd K = P_ * Hj_imu_.transpose() * (Hj_imu_ * P_ * Hj_imu_.transpose() + R_imu).inverse();
    x_ = x_ + K * (z_imu - H_imu_ * x_);
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * Hj_imu_) * P_;
}

Eigen::VectorXd ExtendedKalmanFilter::get_state() const { return x_; }

Eigen::MatrixXd ExtendedKalmanFilter::get_covariance() const { return P_; }

void ExtendedKalmanFilter::linearize_gps()
{
    Hj_gps_ = H_gps_;
}

void ExtendedKalmanFilter::linearize_imu()
{
    Hj_imu_ = H_imu_;
}