// ekf.hpp
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"

constexpr double kEarthRadius = 6371000.0; // Earth radius in meters
constexpr double kDegToRad = M_PI / 180.0;

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


class GPSConversion
{
public:
  GPSConversion::GPSConversion(double ref_latitude, double ref_longitude, double ref_altitude)
    : ref_latitude_(ref_latitude), ref_longitude_(ref_longitude), ref_altitude_(ref_altitude) {}

  Eigen::Vector3d lla_to_enu(double latitude, double longitude, double altitude) const
  {
    Eigen::Vector3d ecef = lla_to_ecef(latitude, longitude, altitude);
    return ecef_to_enu(ecef);
  }

private:
  Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d& ecef) const
  {
    Eigen::Vector3d ref_ecef = lla_to_ecef(ref_latitude_, ref_longitude_, ref_altitude_);

    double lat_rad = ref_latitude_ * kDegToRad;
    double lon_rad = ref_longitude_ * kDegToRad;

    double clat = cos(lat_rad);
    double slat = sin(lat_rad);
    double clon = cos(lon_rad);
    double slon = sin(lon_rad);

    Eigen::Matrix3d R;
    R << -slon, clon, 0,
        -slat * clon, -slat * slon, clat,
        clat * clon, clat * slon, slat;

    return R * (ecef - ref_ecef);
  }
  Eigen::Vector3d lla_to_ecef(double latitude, double longitude, double altitude) const
  {
    double lat_rad = latitude * kDegToRad;
    double lon_rad = longitude * kDegToRad;

    double N = kEarthRadius / sqrt(1.0 - 0.00669437999019758 * pow(sin(lat_rad), 2));

    double x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    double y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    double z = (N * (1 - 0.00669437999019758) + altitude) * sin(lat_rad);

    return Eigen::Vector3d(x, y, z);
  }

  double ref_latitude_;
  double ref_longitude_;
  double ref_altitude_;
};

class PoseEstimator : public rclcpp::Node
{
public:
    PoseEstimator() : Node("pose_estimator"), gps_conv_(0.0, 0.0, 0.0)
    {
        // Initialize the EKF with an initial state and covariance
        Eigen::VectorXd x_init(6);
        x_init << 0, 0, 0, 0, 0, 0;
        Eigen::MatrixXd P_init = Eigen::MatrixXd::Identity(6, 6) * 1e-6;

        ekf_.initialize(x_init, P_init);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&PoseEstimator::imu_callback, this, std::placeholders::_1));
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&PoseEstimator::gps_callback, this, std::placeholders::_1));
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);

        R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 1e-6;
        R_imu_ = Eigen::MatrixXd::Identity(3, 3) * 1e-6;
    }
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract orientation data from the IMU message
        Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
        Eigen::VectorXd z_imu(3);
        z_imu << euler(0), euler(1), euler(2);

        // Update the EKF with IMU data
        ekf_.update_imu(z_imu, R_imu_);

        // Publish the estimated pose
        publish_pose();
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Convert LLA coordinates to ENU
        Eigen::Vector3d enu = gps_conv_.lla_to_enu(msg->latitude, msg->longitude, msg->altitude);

        // Update the EKF with GPS data
        ekf_.update_gps(enu, R_gps_);

        // Publish the estimated pose
        publish_pose();
    }

    void publish_pose()
    {
        // Get the current state and covariance
        Eigen::VectorXd x = ekf_.get_state();
        Eigen::MatrixXd P = ekf_.get_covariance();

        // Create a pose message
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = x(0);
        pose_msg.pose.pose.position.y = x(1);
        pose_msg.pose.pose.position.z = x(2);

        // Create quaternion from roll, pitch, and yaw
        tf2::Quaternion q;
        q.setRPY(x(5), x(4), x(3));
        pose_msg.pose.pose.orientation = tf2::toMsg(q);

        // Convert the covariance matrix to row-major order
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
            pose_msg.pose.covariance[i * 6 + j] = P(i, j);

        // Publish the pose message
        pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    ExtendedKalmanFilter ekf_;
    GPSConversion gps_conv_;

    Eigen::MatrixXd R_gps_;
    Eigen::MatrixXd R_imu_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimator>());
  rclcpp::shutdown();
  return 0;
}