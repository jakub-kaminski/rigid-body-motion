#include "Utils.h"


void Utils::hello(int a) {
    std::cout << "Hi There" << std::endl;
}

Eigen::Matrix4d Utils::localHMat(std::shared_ptr<urdf::Joint> joint, double radAngle) {
    Eigen::Quaterniond q(
            joint->parent_to_joint_transform.rotation.w,
            joint->parent_to_joint_transform.rotation.x,
            joint->parent_to_joint_transform.rotation.y,
            joint->parent_to_joint_transform.rotation.z
    );

    if (joint->type != urdf::JointType::FIXED) {
        auto axisSign = joint->axis.z;
        Eigen::Vector3d rotation(0, 0, radAngle * axisSign);
        double angle = rotation.norm();
        Eigen::Vector3d axis = rotation.normalized();
        Eigen::Quaterniond q_rot(Eigen::AngleAxisd(angle, axis));
        q = q * q_rot;
    }

    Eigen::Matrix3d mat3 = q.toRotationMatrix();
    Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
    mat4.block(0, 0, 3, 3) = mat3;

    mat4(0, 3) = joint->parent_to_joint_transform.position.x;
    mat4(1, 3) = joint->parent_to_joint_transform.position.y;
    mat4(2, 3) = joint->parent_to_joint_transform.position.z;

    return mat4;
}

Eigen::Matrix4d Utils::eulXYZ2HMat(const double roll, const double pitch, const double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block(0,0,3,3) = rotationMatrix;

    return result;
}

Eigen::Vector3d Utils::vectorProjection(Eigen::Vector3d a, Eigen::Vector3d b) {
    return (a.dot(b) / pow(b.norm(), 2)) * b;
}

double Utils::pointToPlaneSignedDistance(Eigen::Vector3d p, Eigen::Matrix4d T) {
    Eigen::Vector4d pH(p.x(), p.y(), p.z(), 1);
    Eigen::Vector4d pHTransformed = T.inverse() * pH;
    return pHTransformed.z();
}

std::pair<Eigen::Vector2d, Eigen::Vector2d>
Utils::my2Dcirc(Eigen::Vector2d p0, double r0, Eigen::Vector2d p1, double r1) {
    Eigen::Vector2d d_vec = p1 - p0;
    double d = d_vec.norm();
    double a = (pow(r0, 2.0) - pow(r1, 2.0) + pow(d, 2.0)) / (2.0 * d);
    double h = sqrt(pow(r0, 2.0) - pow(a, 2.0));

    Eigen::Vector2d p2 = p0 + a * (p1 - p0) / d;

    Eigen::Vector2d p3a(p2.x() + h * (p1.y() - p0.y()) / d,
                        p2.y() - h * (p1.x() - p0.x()) / d);

    Eigen::Vector2d p3b(p2.x() - h * (p1.y() - p0.y()) / d,
                        p2.y() + h * (p1.x() - p0.x()) / d);

    return {p3a, p3b};
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
Utils::my3Dcirc(Eigen::Vector3d c1, double r0, Eigen::Vector3d c2, double r1, Eigen::Matrix4d T) {
    Eigen::Vector4d p1H(c1.x(), c1.y(), c1.z(), 1.0);
    Eigen::Vector4d p2H(c2.x(), c2.y(), c2.z(), 1.0);

    Eigen::Vector4d p1Hplane = T.inverse() * p1H;
    Eigen::Vector4d p2Hplane = T.inverse() * p2H;

//    std::cout << p1Hplane << std::endl;
//    std::cout << p2Hplane << std::endl;

    // approach credits: http://paulbourke.net/geometry/circlesphere/
    Eigen::Vector2d p0(p1Hplane.x(), p1Hplane.y());
    Eigen::Vector2d p1(p2Hplane.x(), p2Hplane.y());

    auto p3 = my2Dcirc(p0,r0,p1,r1);

    Eigen::MatrixXd A(4, 2);
    A << p3.first.x(), p3.second.x(),
            p3.first.y(), p3.second.y(),
            0.0, 0.0,
            1.0, 1.0;

//    std::cout <<"Hello:" << std::endl;
//    std::cout << A << std::endl;

//    pr("A", A);

    Eigen::MatrixXd out(4, 2);
    out = T * A;


//    std::cout << "A value: " <<std::endl;
//    std::cout << A << std::endl;
//    std::cout << "Out value: " <<std::endl;
//    std::cout << out << std::endl;

    Eigen::Vector3d res1(out(0, 0), out(1, 0), out(2, 0));
    Eigen::Vector3d res2(out(0, 1), out(1, 1), out(2, 1));

//    std::cout << res1 << std::endl;
//    std::cout << res2 << std::endl;

    return {res1, res2};
}

double Utils::calculateAdjustment(double er1, double er2, double d_angle) {
    double d_error = er2 - er1;
    return (-er1 / d_error) * d_angle;
}

Eigen::Vector3d Utils::ortoproj(Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d B) {
    Eigen::MatrixXd A(3, 2);
    A << a1.x(), a2.x(),
            a1.y(), a2.y(),
            a1.z(), a2.z();

    Eigen::MatrixXd k = A.transpose()*A;
    Eigen::MatrixXd X = k.inverse()*A.transpose()*B;
    Eigen::Vector3d res = A * X;

    return res;
}

double Utils::goodvecangle(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d planeNormal) {
    double my_ang = atan2(v1.cross(v2).norm(), v1.dot(v2)); // direction agnostic angle
    if(isless(v1.cross(v2).dot(planeNormal), 0.0)) {my_ang = -my_ang;}
    return my_ang;
}

std::vector<double> Utils::matrixToVector(Eigen::Matrix4d mat) {
    std::vector<double> res;
    res.reserve(16);
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            res.push_back(mat(r, c));
        }
    }
    return res;
}

//Eigen::Matrix4d Utils::eulXYZ2HMat(const double roll, const double pitch, const double yaw) {
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
//
//    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
//    Eigen::Matrix3d rotationMatrix = q.matrix();
//    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
//    result.block(0, 0, 3, 3) = rotationMatrix;
//
//    return result;
//}
