#include <Kine.h>

int main(){
    // q_rot
    Eigen::Vector3d rotation(0, 0, EIGEN_PI / 2);
    double angle = rotation.norm();
    Eigen::Vector3d axis = rotation.normalized();
    Eigen::Quaterniond q_rot(Eigen::AngleAxisd(angle, axis));

    Eigen::Matrix3d mat3 = q_rot.toRotationMatrix();
    Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
    mat4.block(0,0,3,3) = mat3;

    std::cout << "MAT4 is: "<< std::endl;
    std::cout << mat4 << std::endl;

    // q
    Eigen::Vector3d rotation1(0, 0, 0);
    double angle1 = rotation1.norm();
    Eigen::Vector3d axis1 = rotation1.normalized();
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle1, axis1));

    Eigen::Matrix3d mat31 = q.toRotationMatrix();
    Eigen::Matrix4d mat41 = Eigen::Matrix4d::Identity();
    mat41.block(0,0,3,3) = mat31;

    std::cout << "MAT41 is: "<< std::endl;
    std::cout << mat41 << std::endl;

    // q rotated by q_rot

//    Eigen::Quaterniond rotatedQ = q_rot * q * q_rot.inverse();
    Eigen::Quaterniond rotatedQ = q_rot * q;
    std::cout << "Rotation with quaternions:"<< std::endl;

    Eigen::Matrix3d rotatedQmat = rotatedQ.toRotationMatrix();
    Eigen::Matrix4d rotatedQmat4 = Eigen::Matrix4d::Identity();
    rotatedQmat4.block(0,0,3,3) = rotatedQmat;
    std::cout << rotatedQmat4 << std::endl;

    std::cout << "Rotation with matrices:"<< std::endl;
    Eigen::Matrix4d rotatedHmat4 = mat41 * mat4;
    std::cout << rotatedHmat4 << std::endl;

    return 0;
}