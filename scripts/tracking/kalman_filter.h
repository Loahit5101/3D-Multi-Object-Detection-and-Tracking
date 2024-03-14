#include <iostream>
#include <Eigen/Core>
#include  <string>
class ExtendedKalmanFilter {

private:
    int state_dim;
    double dt;
    double process_variance;

public:
    ExtendedKalmanFilter(int state_dim, double dt, double process_variance) 
        : state_dim(state_dim), dt(dt), process_variance(process_variance) {}

    Eigen::MatrixXd get_F() {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim, state_dim);
        F(0, 3) = F(1, 4) = F(2, 5) = dt;
        return F;
    }
    
    Eigen::MatrixXd get_Q() {

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim, state_dim);
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
 
        Q(0, 0) = Q(1, 1) = Q(2, 2) = dt4 / 4;
        Q(0, 3) = Q(1, 4) = Q(2, 5) = dt3 / 2;
        Q(3, 0) = Q(4, 1) = Q(5, 2) = dt3 / 2;
        Q(3, 3) = Q(4, 4) = Q(5, 5) = dt2;
        
        /*
        Q << dt4 / 4.0, 0, 0, dt3 / 2.0, 0, 0,
             0, dt4 / 4.0, 0, 0, dt3 / 2.0, 0,
             0, 0, dt4 / 4.0, 0, 0, dt3 / 2.0,
             dt3 / 2.0, 0, 0, dt2, 0, 0,
             0, dt3 / 2.0, 0, 0, dt2, 0,
             0, 0, dt3 / 2.0, 0, 0, dt2;*/

        return Q;
    }

    void predict(Eigen::VectorXd& x, Eigen::MatrixXd& P) {
      
        Eigen::MatrixXd F = get_F();
        Eigen::MatrixXd Q = get_Q();
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(Eigen::VectorXd& x, Eigen::MatrixXd& P, Eigen::MatrixXd& z, std::string sensor_name){

         Eigen::MatrixXd H;
         Eigen::MatrixXd y;
         Eigen::MatrixXd R(3,3);

        if (sensor_name == "LiDAR"){

          int measurement_dim = 3;
          H = Eigen::MatrixXd::Zero(measurement_dim, state_dim);

          H(0,0)=1;
          H(1,1)=1;
          H(2,2)=1;

          std::cout<<H;
          

          Eigen::MatrixXd x_pos(3, 1); // get x,y and z from x
          x_pos << x(1,0),x(2,0),x(3,0); // Example LiDAR measurement

           y = z - x_pos;  // Lidar measurement function is linear(we get position from detection directly)
          
          double s_x = 0.1;
          double s_y = 0.1;
          double s_z = 0.1;

          R << s_x*s_x, 0, 0, 
               0, s_y*s_y, 0,
               0, 0, s_z*s_z;

        }

        else if(sensor_name == "camera"){

          //TODO
        }

          Eigen::MatrixXd S = H * P * H.transpose() + R ; 

          Eigen::MatrixXd K = P * H.transpose() * S.inverse();

          x = x + (K * y);

          Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
          P = (I - K * H) * P;

    }
};
