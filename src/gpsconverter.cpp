#include <gps_acc_fusion/gpsconverter.hpp>


Eigen::Vector3d GPSConversion::lla_to_enu(double latitude, double longitude, double altitude) const
{
    Eigen::Vector3d ecef = lla_to_ecef(latitude, longitude, altitude);
    return ecef_to_enu(ecef);
}

Eigen::Vector3d GPSConversion::ecef_to_enu(const Eigen::Vector3d& ecef) const
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

Eigen::Vector3d GPSConversion::lla_to_ecef(double latitude, double longitude, double altitude) const
{
    double lat_rad = latitude * kDegToRad;
    double lon_rad = longitude * kDegToRad;

    double N = kEarthRadius / sqrt(1.0 - 0.00669437999019758 * pow(sin(lat_rad), 2));

    double x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    double y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    double z = (N * (1 - 0.00669437999019758) + altitude) * sin(lat_rad);

    return Eigen::Vector3d(x, y, z);
}
