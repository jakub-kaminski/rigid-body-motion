#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
#include <memory>

#include "urdf/model.h"



namespace Utils {
    void hello(int a);

    Eigen::Matrix4d localHMat(std::shared_ptr<urdf::Joint> joint, double radAngle);

//Eigen::Matrix4d
    Eigen::Matrix4d eulXYZ2HMat(double, double, double);

    Eigen::Vector3d vectorProjection(Eigen::Vector3d a, Eigen::Vector3d b);

    double pointToPlaneSignedDistance(Eigen::Vector3d p, Eigen::Matrix4d T);

    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    my3Dcirc(Eigen::Vector3d c1, double r0, Eigen::Vector3d c2, double r1, Eigen::Matrix4d T);

    std::pair<Eigen::Vector2d, Eigen::Vector2d>
    my2Dcirc(Eigen::Vector2d p0, double r0, Eigen::Vector2d p1, double r1);

    double calculateAdjustment(double er1, double er2, double d_angle);

    Eigen::Matrix4d eulXYZ2HMat(double roll, double pitch, double yaw);

    Eigen::Vector3d ortoproj(Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d B);

    double goodvecangle(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d planeNormal);

    std::vector<double> matrixToVector(Eigen::Matrix4d mat);

}

#endif //UTILS_H
