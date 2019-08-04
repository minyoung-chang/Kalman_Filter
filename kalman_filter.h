#ifndef kalman_filter_hpp
#define kalman_filter_hpp

#include "Eigen/Dense"

class KalmanFilter {
public:
    KalmanFilter();
    
    virtual ~KalmanFilter();
    
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
              Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);
    
    void Predict();
    void Update(const Eigen::VectorXd &z);
    void UpdateEKF(const Eigen::VectorXd &z);
    
    Eigen::VectorXd x_;
    Eigen::VectorXd x_predict_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd Q_;
};

#endif /* kalman_filter_hpp */
