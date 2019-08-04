#ifndef measurement_package_h
#define measurement_package_h

#include "Eigen/Dense"

class MeasurementPackage {
public:
    enum SensorType{
        LIDAR,
        RADAR
    } sensor_type_;
    
    long long timestamp_;
    Eigen::VectorXd raw_measurements_;
};

#endif /* measurement_package_h */
