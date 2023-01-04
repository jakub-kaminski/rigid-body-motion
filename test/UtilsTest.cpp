#include "Utils.h"


void pr(std::string s, Eigen::Matrix4d T){
    std::cout <<"Matrix " << s << ": " << std::endl;
    std::cout << T << std::endl;
}

void pr(std::string s, Eigen::Vector3d v){
    std::cout <<"Vector " << s << ": " << std::endl;
    std::cout << v << std::endl;
}

void pr(std::string s, Eigen::Vector2d v){
    std::cout <<"Vector 2D " << s << ": " << std::endl;
    std::cout << v << std::endl;
}

void pr(std::string s, double d){
    std::cout <<"Value " << s << ": " << std::endl;
    std::cout << d << std::endl;
}

int main(){
//    Eigen::Vector3d p0{0.0, 0.0, 0.0};
//    double r0 = 4.0;
//    Eigen::Vector3d p1{4.0, 4.0, 0.0};
//    double r1 = 3.0;

//       1.005174493308272   3.938958008556091
//   3.576932287878275   0.643148772630457
//  -1.481613865329406  -0.266400944247145

//    Eigen::Matrix4d T;
//    T << 0.939692620785908,  -0.342020143325669, 0.0, 0.0,
//   0.315985410125162, 0.868162779195991, 0.382683432365090, 0,
//  -0.130885442385867, -0.359604797490498, 0.923879532511287, 0,
//   0, 0, 0, 1.000000000000000;


//    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

//    auto pt_out = Utils::my3Dcirc(p0, r0, p1, r1, T);


//    Eigen::Vector3d p0{0.0, 0.0, 10.0};
//    double r0 = 3.0;
//    Eigen::Vector3d p1{2.0, 6.0, 10.0};
//    double r1 = 5.0;
//
//    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//    T.block(0,3,3,1) = p1;
//
//    auto pt_out = Utils::my3Dcirc(p0, r0, p1, r1, T);


    Eigen::Vector2d p0{0.0, 0.0};
    double r0 = 3.0;
    Eigen::Vector2d p1{2.0, 6.0};
    double r1 = 5.0;

    auto pt_out = Utils::my2Dcirc(p0, r0, p1, r1);

    pr("pt_out.first", pt_out.first);
    pr("pt_out.second", pt_out.second);

    return 0;
}