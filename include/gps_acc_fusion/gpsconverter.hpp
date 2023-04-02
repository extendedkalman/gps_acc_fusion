#include <cmath>
#include <Eigen/Dense>

constexpr double kEarthRadius = 6371000.0; // Earth radius in meters
constexpr double kDegToRad = M_PI / 180.0;


class GPSConversion
{
public:
  GPSConversion(double ref_latitude, double ref_longitude, double ref_altitude): ref_latitude_(ref_latitude), ref_longitude_(ref_longitude), ref_altitude_(ref_altitude)
  {}

  Eigen::Vector3d lla_to_enu(double latitude, double longitude, double altitude) const;

private:
  Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d& ecef) const;

  Eigen::Vector3d lla_to_ecef(double latitude, double longitude, double altitude) const;

  double ref_latitude_;
  double ref_longitude_;
  double ref_altitude_;
};