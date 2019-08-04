#include "Fusion.h"
#include <iostream>
#include "Eigen/Dense"
#include "Tools.h"

/**
 * Constructor
 */
Fusion::Fusion() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_lidar_ = Eigen::MatrixXd(2, 2);
    R_radar_ = Eigen::MatrixXd(3, 3);
    H_lidar_ = Eigen::MatrixXd(2, 4);
    H_radar_ = Eigen::MatrixXd(3, 4);
    
    //measurement covariance matrix - laser
    R_lidar_ << 0.0225, 0,
                0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    
    // measurement function H
    H_lidar_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    H_radar_ << 1, 1, 0, 0,
                1, 1, 0, 0,
                1, 1, 1, 1;
    
    //state covariance matrix P
    kf_.P_ = Eigen::MatrixXd(4, 4);
    kf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    
    //the initial transition matrix F_
    kf_.F_ = Eigen::MatrixXd(4, 4);
    kf_.F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
}

/**
 * Destructor
 */
Fusion::~Fusion() {}


/**
 * if not initialized, initialize the state ekf_.x_ with the first measurement
 * create the covariance matrix.
 */
void Fusion::ProcessMeasurement(const MeasurementPackage &measurement_package) {
    if (!is_initialized_) {
        kf_.x_ = Eigen::VectorXd(4);
        kf_.x_ << 0.6,0.6,1,1;  // initial state
        kf_.x_predict_ = Eigen::VectorXd(4);
        kf_.x_predict_ << 0.6,0.6,1,1;  // initial prediction
        
        if (measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = measurement_package.raw_measurements_(0);
            double phi = measurement_package.raw_measurements_(1);
            double rho_dot = measurement_package.raw_measurements_(2);
            
            kf_.x_(0) = rho * cos(phi);     // x
            kf_.x_(1) = rho * sin(phi);     // y
            kf_.x_(2) = rho_dot * cos(phi);     // vx
            kf_.x_(3) = rho_dot * sin(phi);     // vy
        }
        else if (measurement_package.sensor_type_ == MeasurementPackage::LIDAR) {
            kf_.x_(0) = measurement_package.raw_measurements_(0);
            kf_.x_(1) = measurement_package.raw_measurements_(1);
        }
        
        is_initialized_ = true;
        std::cout << "System Initialized" << std::endl << std::endl;
        return;
    }   // if (!is_initialized_)
    
    
    /**
     * Predict with state
     */
    //compute the time elapsed between the current and previous measurements
    double dt = (measurement_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_package.timestamp_;      // update
    
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    
    //Modify the F matrix so that the time is integrated
    kf_.F_(0, 2) = dt;
    kf_.F_(1, 3) = dt;
    
    //set the acceleration noise components
    double noise_ax = 9;
    double noise_ay = 9;
    
    //set the process covariance matrix Q
    kf_.Q_ = Eigen::MatrixXd(4, 4);
    kf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
              0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
              dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
    
    kf_.Predict();
    kf_.x_predict_ = kf_.x_;
    
    /**
     * Update with measurement
     */
    if (measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        Tools tools;
        H_radar_ = tools.CalculateJacobian(kf_.x_);
        kf_.H_ = H_radar_;
        kf_.R_ = R_radar_;
        kf_.UpdateEKF(measurement_package.raw_measurements_);
        
    } else {
        // Laser updates
        kf_.H_ = H_lidar_;
        kf_.R_ = R_lidar_;
        kf_.Update(measurement_package.raw_measurements_);
    }
    
    // print the output
    //std::cout << "x_ = " << kf_.x_ << std::endl;
    //std::cout << "P_ = " << kf_.P_ << std::endl;
}
