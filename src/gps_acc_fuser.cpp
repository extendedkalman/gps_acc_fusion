#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "Eigen/Dense"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>


using std::placeholders::_1;

class GPSAccHandler : public rclcpp::Node
{
  public:
    GPSAccHandler(): Node("GPSAccHandler")
    {
        prev_enu_pos.setZero();
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("demo/imu", 10, std::bind(&GPSAccHandler::imu_callback, this, _1));
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps", 10, std::bind(&GPSAccHandler::gps_callback, this, _1));

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

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        

        if (counter_ == 0) {
            phi0 = msg->latitude;           
            lam0 = msg->longitude; 
            h0 = msg->altitude;
            Eigen::VectorXd initial_ecef = this->transform_LLA_to_ECEF(phi0, lam0, h0);
            x0 = initial_ecef(0);
            y0 = initial_ecef(1);
            z0 = initial_ecef(2);
            counter_ = counter_ + 1;
        }

        Eigen::VectorXd ecef_pos = this->transform_LLA_to_ECEF(msg->latitude, msg->longitude, msg->altitude);
        Eigen::VectorXd enu_pos = this->transform_ECEF_TO_ENU(ecef_pos(0), ecef_pos(1), ecef_pos(2));
        
        RCLCPP_INFO(this->get_logger(), "ENU Pos x: '%f'", enu_pos(0));
        RCLCPP_INFO(this->get_logger(), "ENU Pos y: '%f'", enu_pos(1));
        RCLCPP_INFO(this->get_logger(), "ENU Pos z: '%f'", enu_pos(2));


        Eigen::VectorXd enu_vec = (enu_pos - prev_enu_pos) / 1.0;
        RCLCPP_INFO(this->get_logger(), "ENU vec x: '%f'", enu_vec(0));
        RCLCPP_INFO(this->get_logger(), "ENU vec y: '%f'", enu_vec(1));
        RCLCPP_INFO(this->get_logger(), "ENU vec z: '%f'", enu_vec(2));
        


        std::array<double, 9> gps_pos_covariance = msg->position_covariance;

        prev_enu_pos = enu_pos;
    }

    Eigen::VectorXd transform_LLA_to_ECEF(double phi, double lam, double h)
    {
        double a = 6378137.0;                   // WGS84 semi-major axis (meters)
        double f = 1/298.257223563;             // WGS84 flattening
        double b = a * (1 - f);                 // WGS84 semi-minor axis (meters)
        //double e_sq = (std::pow(a, 2) - std::pow(b, 2)) / std::pow(a, 2);     // Eccentricity squared
        double e_sq = 1 - (std::pow(b, 2) / std::pow(a, 2));
        //phi = latitude in radians
        //lam = longitude in radians
        //h = altitude in meters

        double N = a / std::sqrt(1 - e_sq * std::pow(std::sin(phi), 2));

        double x = (N + h) * std::cos(phi) * std::cos(lam);
        double y = (N + h) * std::cos(phi) * std::sin(lam);
        double z = ((1 - e_sq) * N + h) * std::sin(phi);
        ecef_coordinates << x, y, z;
        return ecef_coordinates;
    }

    Eigen::VectorXd transform_ECEF_TO_ENU(double x, double y, double z)
    {

        double r0 = sqrt(std::pow(x0, 2) + std::pow(y0, 2) + std::pow(z0, 2));

        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        double sphi0 = std::sin(phi0);
        double phi0 = std::cos(phi0);
        double slam0 = std::sin(lam0);
        double clam0 = std::cos(lam0);

        double azimuth = std::atan2(-slam0, clam0);
        double elevation = std::atan2(sphi0, std::sqrt(std::pow(clam0, 2) + std::pow(slam0, 2)));

        double sA = std::sin(azimuth);
        double cA = std::cos(azimuth);
        double sE = std::sin(elevation);
        double cE = std::cos(elevation);

        double dEast = -sA*dx + cA*dy;
        double dNorth = -cA*sE*dx - sA*sE*dy + cE*dz;
        double dUp = cA*cE*dx + sA*cE*dy + sE*dz;

        enu_coordinates << dEast, dNorth, dUp;

        return enu_coordinates;
    }


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    Eigen::VectorXd state_ = Eigen::VectorXd(4);
    Eigen::VectorXd ecef_coordinates = Eigen::VectorXd(3);
    Eigen::VectorXd enu_coordinates = Eigen::VectorXd(3);
    Eigen::VectorXd prev_enu_pos = Eigen::VectorXd(3); 
    bool output_ = false;
    double counter_ = 0;
    double x0, y0, z0; //reference point ECEF coordinates
    double phi0;             //reference point LLA coordinates
    double lam0; 
    double h0;   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSAccHandler>());
  rclcpp::shutdown();
  return 0;
}