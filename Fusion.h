#ifndef Fusion_hpp
#define Fusion_hpp

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "Tools.h"

class Fusion {
    bool is_initialized_;
    long long previous_timestamp_;
    Tools tools;
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_lidar_;
    Eigen::MatrixXd H_radar_;
    
public:
    Fusion();
    virtual ~Fusion();
    KalmanFilter kf_;
    
    void ProcessMeasurement(const MeasurementPackage &measurement_package);


};



#endif /* Fusion_hpp */
