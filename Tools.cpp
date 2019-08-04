#include <stdio.h>
#include <iostream>
#include "Tools.h"

Tools::Tools() {}
Tools::~Tools() {}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state) {
    /**
     * x_state = px, py, vx, vy
     */
    Eigen::MatrixXd Hj(3,4);
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    
    // pre-calculate to avoid repetition
    double constant1 = px*px + py*py;
    double constant2 = sqrt(constant1);
    double constant3 = constant1 * constant2;
    
    // check division by zero
    if (constant1 < 0.0001)
    {
        px += 0.0001;
        py += 0.0001;
        constant1 = px*px+py*py;
    }
    
    Hj <<  (px/constant2), (py/constant2), 0, 0,
          -(py/constant1), (px/constant1), 0, 0,
           py*(vx*py - vy*px)/constant3, px*(px*vy - py*vx)/constant3, px/constant2, py/constant2;
    
    return Hj;
}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) {
    
    Eigen::VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    if (estimations.size() != ground_truth.size()
     || estimations.size() == 0) {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }
    
    for (unsigned int i=0; i < estimations.size(); ++i) {
        Eigen::VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    
    return rmse;
}
