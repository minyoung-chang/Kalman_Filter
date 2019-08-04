#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {}
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
    // predict with previous state
    x_ = x_in;  // state
    x_predict_ = x_in;
    P_ = P_in;  // state covariance
    F_ = F_in;  // state transition matrix
    
    // update with measurement
    H_ = H_in;  // measurement function
    R_ = R_in;  // measurement covariance (noise)
    
    Q_ = Q_in;  // process covariance matrix
}

void KalmanFilter::Predict() {
    // x' = Fx + noise(ignored here)
    x_ = F_ * x_;
    // P' = (F P F^T) + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
    Eigen::VectorXd z_pred = H_ * x_;
    Eigen::VectorXd y = z - z_pred;     // error
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd K = PHt * Si;   // kalman gain
    
    // new estimate
    x_ = x_ + (K*y);
    long x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
    // calculate based on the current state
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double phi = atan2(x_(1), x_(0));
    double rho_dot;
    if (fabs(rho) < 0.0001) {
        rho_dot = 0;
    } else {
        rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
    }
    Eigen::VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    Eigen::VectorXd y = z - z_pred;
    
    // normalizing phi to be within -pi and pi
    // atan2() returns values between -pi and pi
    while ( y(1) > M_PI || y(1) < -M_PI )
    {
        if ( y(1) > M_PI )
        {
            y(1) -= M_PI;
        }else {
            y(1) += M_PI;
        }
    }

    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd K = PHt * Si;

    // new estimate
    x_ = x_ + (K*y);
    long x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_) * P_;
}
