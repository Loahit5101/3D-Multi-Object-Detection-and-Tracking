#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <Eigen/Core>
#include  <string>
class ExtendedKalmanFilter {

private:
    int state_dim;
    int measurement_dim;
    double process_variance;

    Eigen::VectorXd x_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd P_initial;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;

public:
    
    ExtendedKalmanFilter() 
    {

         double s_x = 0.1;
         double s_y = 0.1;
         double s_z = 0.1;

         process_variance = 0.01;

         measurement_dim = 3;
         state_dim = 6;
         
          x_ = Eigen::VectorXd::Zero(state_dim);
          F_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
          Q_ = Eigen::MatrixXd::Zero(state_dim, state_dim);
          P_initial = Eigen::MatrixXd::Zero(state_dim, state_dim);
          P_ = Eigen::MatrixXd::Zero(state_dim, state_dim);
          R_ = Eigen::MatrixXd::Zero(measurement_dim, measurement_dim);
          R_ << s_x*s_x, 0, 0, 
               0, s_y*s_y, 0,
               0, 0, s_z*s_z;

          H_ = Eigen::MatrixXd::Zero(measurement_dim, state_dim);

          H_(0,0)=1;
          H_(1,1)=1;
          H_(2,2)=1;     

    }

    void init_x(Eigen::Vector3f position) {
        
       // set initial state with first measurement 
       x_ <<position.x(),position.y(),position.z(),0.0,0.0,0.0; 
    
    }

    void update_F(double dt) {
        
        F_(0, 3) = F_(1, 4) = F_(2, 5) = dt;
    
    }
    
    void update_Q(double dt) {

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
 
        Q_(0, 0) = Q_(1, 1) = Q_(2, 2) = dt4 / 4;
        Q_(0, 3) = Q_(1, 4) = Q_(2, 5) = dt3 / 2;
        Q_(3, 0) = Q_(4, 1) = Q_(5, 2) = dt3 / 2;
        Q_(3, 3) = Q_(4, 4) = Q_(5, 5) = dt2;
        
    }

    void predict(double dt) {
      
        update_F(dt);
        update_Q(dt);
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(Eigen::VectorXd& x, Eigen::MatrixXd& P, Eigen::MatrixXd& z, std::string sensor_name){

        Eigen::MatrixXd y;

        if (sensor_name == "LiDAR"){

          Eigen::MatrixXd x_pos(3, 1); // get x,y and z from x
          x_pos << x_(1,0),x_(2,0),x_(3,0);

          
          y = z - x_pos;  // Lidar measurement function is linear(we get position from detection directly)
          
        }

        else if(sensor_name == "camera"){

          //TODO
        }
        
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_ ; 
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);

        x_ = x_ + (K * y);
        P_ = (I - K * H_) * P_;
    }
};

#endif