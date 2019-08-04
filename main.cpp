#include <iostream>
#include <fstream>
#include <string>
#include "Eigen/Dense"
#include "Tools.h"
#include "measurement_package.h"
#include <math.h>
#include "Fusion.h"

int main(int argc, const char * argv[]) {
    
    int cnt = 0;

    std::string line;
    std::ifstream DATA_TEXT ("data/ `");
    std::ofstream OUTPUT_PD ("data/output_predict.txt");
    std::ofstream OUTPUT_KF ("data/output_kf.txt");
    std::ofstream OUTPUT_GT ("data/output_gt.txt");
    std::ofstream OUTPUT_LIDAR ("data/output_lidar.txt");
    std::ofstream OUTPUT_RADAR ("data/output_radar.txt");
    std::ofstream OUTPUT_RMSE ("data/output_rmse.txt");
    
    /** data layout
     * LIDAR: sensor_type, x_measured,   y_measured,                    timestamp, x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yawrate_gt               //  9 cols
     * RADAR: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yawrate_gt               // 10 cols
     */
    
    Fusion fusion;
    Tools tools;
    
    // store values for calculating RMSE later
    std::vector<Eigen::VectorXd> estimations;
    std::vector<Eigen::VectorXd> ground_truth;
    
    
    if (DATA_TEXT.is_open()) {
        while ( getline(DATA_TEXT, line) )    // read in the data line by line
        {
            // print out count number
            std::cout << std::endl << "===== " << cnt << " =====" << std::endl;
            
            std::istringstream iss(line);   // segment the data
            
            MeasurementPackage measurement_package;     // for storing the received input
            
            std::string sensor_type;   // first
            iss >> sensor_type;
            
            if (sensor_type.compare("L") == 0) {  // if it is LIDAR data
                measurement_package.sensor_type_ = MeasurementPackage::LIDAR;
                
                double px; double py;
                iss >> px; iss >> py;
                measurement_package.raw_measurements_ = Eigen::VectorXd(2);
                measurement_package.raw_measurements_ << px, py;
                
                long long timestamp;
                iss >> timestamp;
                measurement_package.timestamp_ = timestamp - 1477010443000000;
                OUTPUT_LIDAR << px << " " << py << std::endl;
                
            } else if (sensor_type.compare("R") == 0) {
                measurement_package.sensor_type_ = MeasurementPackage::RADAR;
                
                double rho; double phi; double rho_dot;
                iss >> rho; iss >> phi; iss >> rho_dot;
                measurement_package.raw_measurements_ = Eigen::VectorXd(3);
                measurement_package.raw_measurements_ << rho, phi, rho_dot;
                
                long long timestamp;
                iss >> timestamp;
                measurement_package.timestamp_ = timestamp - 1477010443000000;
                
                double px; double py;
                px = rho * cos(phi);     // x
                py = rho * sin(phi);     // y
                
                OUTPUT_RADAR << px << " " << py << std::endl;
            }
            
            double x_gt; double y_gt; double vx_gt; double vy_gt;
            iss >> x_gt; iss >> y_gt; iss >> vx_gt; iss >> vy_gt;
            
            Eigen::VectorXd gt_values;
            gt_values = Eigen::VectorXd(4);
            gt_values << x_gt, y_gt, vx_gt, vy_gt;
            ground_truth.push_back(gt_values);
            fusion.ProcessMeasurement(measurement_package);
            
            Eigen::VectorXd prediction(4);
            double p_x_pred = fusion.kf_.x_predict_(0);
            double p_y_pred = fusion.kf_.x_predict_(1);
            double v_x_pred = fusion.kf_.x_predict_(2);
            double v_y_pred = fusion.kf_.x_predict_(3);
            prediction(0) = p_x_pred;
            prediction(1) = p_y_pred;
            prediction(2) = v_x_pred;
            prediction(3) = v_y_pred;

            Eigen::VectorXd estimation(4);
            double p_x = fusion.kf_.x_(0);
            double p_y = fusion.kf_.x_(1);
            double v1  = fusion.kf_.x_(2);
            double v2 = fusion.kf_.x_(3);
            estimation(0) = p_x;
            estimation(1) = p_y;
            estimation(2) = v1;
            estimation(3) = v2;
            
            estimations.push_back(estimation);
            Eigen::VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
            std::cout << std::endl << "x = " << estimation << std::endl;
            std::cout << std::endl << "gt = " << gt_values << std::endl;
            std::cout << std::endl << "RMSE = " << RMSE << std::endl;
            OUTPUT_PD << prediction(0) << " " << prediction(1) << " " << prediction(2) << " " << prediction(3) << std::endl;
            OUTPUT_KF << estimation(0) << " " << estimation(1) << " " << estimation(2) << " " << estimation(3) << std::endl;
            OUTPUT_GT << gt_values(0) << " " << gt_values(1) << " " << gt_values(2) << " " << gt_values(3) << std::endl;
            OUTPUT_RMSE << RMSE(0) << " " << RMSE(1) << " " << RMSE(2) << " " << RMSE(3) << std::endl;
            cnt ++;
            
            
        //while ( getline(DATA_TEXT, line) )
        }
        DATA_TEXT.close();
        OUTPUT_KF.close();
        OUTPUT_GT.close();
        OUTPUT_PD.close();
        OUTPUT_LIDAR.close();
        OUTPUT_RADAR.close();
        OUTPUT_RMSE.close();
    //if (DATA_TEXT.is_open())
        
    } else {
        std::cout << "file is not opened" << std::endl;
    }
    
    return 0;
}
