#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Dense"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>


using std::placeholders::_1;

class GPSAccHandler : public rclcpp::Node
{
  public:
    GPSAccHandler(): Node("GPSAccHandler")
    {
        
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("demo/imu", 10, std::bind(&GPSAccHandler::imu_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("demo/odom", 10, std::bind(&GPSAccHandler::gps_callback, this, _1));
    }


  private:

    

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(output_)
        {
            RCLCPP_INFO(this->get_logger(), "#######################IMU#######################");

            RCLCPP_INFO(this->get_logger(), "IMU: header frame id: '%s'", msg->header.frame_id);
            RCLCPP_INFO(this->get_logger(), "IMU: header stamp nanosec: '%d'", msg->header.stamp.nanosec);
            RCLCPP_INFO(this->get_logger(), "IMU: header stamp sec: '%d'", msg->header.stamp.sec);

            RCLCPP_INFO(this->get_logger(), "IMU: linear acc x componenet: '%f'", msg->linear_acceleration.x);
            RCLCPP_INFO(this->get_logger(), "IMU: linear acc y componenet: '%f'", msg->linear_acceleration.y);
            RCLCPP_INFO(this->get_logger(), "IMU: linear acc z componenet: '%f'", msg->linear_acceleration.z);
            for (int i = 0; i < 9; i++)
            {
                RCLCPP_INFO(this->get_logger(), "IMU: linear acc covariance: '%f'", msg->linear_acceleration_covariance[i]);
            }

            RCLCPP_INFO(this->get_logger(), "IMU: angular velocity x component: '%f'", msg->angular_velocity.x);
            RCLCPP_INFO(this->get_logger(), "IMU: angular velocity y component: '%f'", msg->angular_velocity.y);
            RCLCPP_INFO(this->get_logger(), "IMU: angular velocity z component: '%f'", msg->angular_velocity.z);
            for (int i = 0; i < 9; i++)
            {
                RCLCPP_INFO(this->get_logger(), "IMU: linear angular velocity covariance: '%f'", msg->angular_velocity_covariance[i]);
            }

            RCLCPP_INFO(this->get_logger(), "IMU: quaternion x component: '%f'", msg->orientation.x);
            RCLCPP_INFO(this->get_logger(), "IMU: quaternion y component: '%f'", msg->orientation.y);
            RCLCPP_INFO(this->get_logger(), "IMU: quaternion z component: '%f'", msg->orientation.z);
            RCLCPP_INFO(this->get_logger(), "IMU: quaternion w component: '%f'", msg->orientation.w);

            for (int i = 0; i < 9; i++)
            {
                RCLCPP_INFO(this->get_logger(), "IMU: linear orientation covariance: '%f'", msg->orientation_covariance[i]);
            }
        }

        
        
    }

    void gps_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        
    }

    Eigen::VectorXd transform_LLA_to_ECEF(double phi, double lam, double h)
    {
        double a = 6378137.0;                   // WGS84 semi-major axis (meters)
        double f = 1/298.257223563;             // WGS84 flattening
        double b = a * (1 - f);                 // WGS84 semi-minor axis (meters)
        double e_sq = (std::pow(a, 2) - std::pow(b, 2)) / std::pow(a, 2);     // Eccentricity squared
 
        //phi = latitude in radians
        //lam = longitude in radians
        //h = altitude in meters

        double N = a / sqrt(1 - e_sq *  std::pow(sin(phi), 2));

        double x = (N + h) * cos(phi) * cos(lam);
        double y = (N + h) * cos(phi) * sin(lam);
        double z = ((1 - e_sq) * N + h) * sin(phi);
        ecef_coordinates << x, y, z;
        return ecef_coordinates;
    }

    Eigen::VectorXd transform_ECEF_TO_ENU(double x, double y, double z)
    {
        double x0, y0, z0 = 0.0; //reference point ECEF coordinates
        double phi0;             //reference point LLA coordinates
        double lam0; 
        double h0;   

        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        double sphi0 = sin(phi0);
        double cphi0 = cos(phi0);
        double slam0 = sin(lam0);
        double clam0 = cos(lam0);

        double dEast = -slam0*dx + clam0*dy;
        double dNorth = -slam0*cphi0*dx - clam0*cphi0*dy + sphi0*dz;
        double dUp = clam0*cphi0*dx + slam0*cphi0*dy + sphi0*cphi0*dz;

        enu_coordinates << dEast, dNorth, dUp;

        return enu_coordinates;
    }


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    Eigen::VectorXd state_ = Eigen::VectorXd(4);
    Eigen::VectorXd ecef_coordinates = Eigen::VectorXd(3);
    Eigen::VectorXd enu_coordinates = Eigen::VectorXd(3);
    bool output_ = false;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSAccHandler>());
  rclcpp::shutdown();
  return 0;
}