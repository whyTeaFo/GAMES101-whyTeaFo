#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    const double angle = 45.0/180.0*acos(-1);
    Eigen::Vector3d P(2,1,1);
    //rotate matrix
    Eigen::Matrix3d R;
    R << cos(angle), -sin(angle), 0,
         sin(angle),  cos(angle), 0,
         0,           0,          1;
    //translate matrix
    Eigen::Matrix3d T;
    T << 1, 0, 1,
         0, 1, 2,
         0, 0, 1;
    Eigen::Vector3d Q = T * R * P;
    std::cout << "Result of transformation of P: (" << Q(0) << ", " << Q(1) << ")\n";
    return 0;
}