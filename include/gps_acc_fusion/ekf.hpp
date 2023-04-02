#include <Eigen/Dense>

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter();

  void initialize(const Eigen::VectorXd& x, const Eigen::MatrixXd& P);

  void predict(const Eigen::VectorXd& u, double dt);

  void update_gps(const Eigen::VectorXd& z_gps, const Eigen::MatrixXd& R_gps);

  void update_imu(const Eigen::VectorXd& z_imu, const Eigen::MatrixXd& R_imu);

  Eigen::VectorXd get_state() const;
  Eigen::MatrixXd get_covariance() const;

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

  void linearize_gps();

  void linearize_imu();
};