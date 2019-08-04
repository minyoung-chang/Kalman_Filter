#ifndef Tools_h
#define Tools_h

#include <vector>
#include "Eigen/Dense"

class Tools {
public:
    Tools();
    
    virtual ~Tools();
    
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
    
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                  const std::vector<Eigen::VectorXd> &ground_truth);

};

#endif /* Tools_h */
