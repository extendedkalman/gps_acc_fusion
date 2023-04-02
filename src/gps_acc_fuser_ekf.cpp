#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <gps_acc_fusion/ekf.hpp> 
#include <gps_acc_fusion/gpsconverter.hpp>



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
            "/demo/imu", 10, std::bind(&PoseEstimator::imu_callback, this, std::placeholders::_1));
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&PoseEstimator::gps_callback, this, std::placeholders::_1));
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);

        R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 1e-6;
        R_imu_ = Eigen::MatrixXd::Identity(3, 3) * 1e-6;
    }
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double last_imu_time = msg->header.stamp.sec;

        // Calculate the time difference
        double dt = (msg->header.stamp.sec - last_imu_time);

        // Get the angular velocities from the IMU message
        Eigen::VectorXd u(6);
        u << 0, 0, 0, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

        // Call the predict function with the control input and time difference
        ekf_.predict(u, dt);

        // Update the IMU with the new linear accelerations and covariance
        Eigen::VectorXd z_imu(3);
        z_imu << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

        Eigen::MatrixXd R_imu(3, 3);
        R_imu << msg->linear_acceleration_covariance[0], msg->linear_acceleration_covariance[1], msg->linear_acceleration_covariance[2],
                msg->linear_acceleration_covariance[3], msg->linear_acceleration_covariance[4], msg->linear_acceleration_covariance[5],
                msg->linear_acceleration_covariance[6], msg->linear_acceleration_covariance[7], msg->linear_acceleration_covariance[8];

        ekf_.update_imu(z_imu, R_imu);

        // Publish the estimated pose
        publish_pose();

        last_imu_time = msg->header.stamp.sec;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Convert the GPS LLA coordinates to ECEF coordinates
        Eigen::Vector3d enu = gps_conv_.lla_to_enu(msg->latitude, msg->longitude, msg->altitude);

        // Update the EKF with the new GPS measurements and covariance
        Eigen::VectorXd z_gps(3);
        z_gps << enu(0), enu(1), enu(2);

        Eigen::MatrixXd R_gps(3, 3);
        R_gps << msg->position_covariance[0], msg->position_covariance[1], msg->position_covariance[2],
                msg->position_covariance[3], msg->position_covariance[4], msg->position_covariance[5],
                msg->position_covariance[6], msg->position_covariance[7], msg->position_covariance[8];

        ekf_.update_gps(z_gps, R_gps);

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