#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#include "Kine.h"

class RobotInterface {
    Eigen::Matrix4d adjust = Eigen::Matrix4d::Identity();
    std::map<std::string, std::vector<double>> serializedMap;
public:
    std::map<std::string, std::shared_ptr<RBLink>> RBMap;
    RobotInterface() = default;
    RobotInterface(std::map<std::string, std::shared_ptr<RBLink>> RBMap, Eigen::Matrix4d adjust);
    ~RobotInterface() = default;
    std::map<std::string, std::vector<double>> getSerializedTransforms();
    void setAdjustTransform(Eigen::Matrix4d mat);
    std::vector<double> getTransformByID(std::string linkName);

    std::vector<double> getVectorFromMatrix(Eigen::Matrix4d mat);
};


#endif //ROBOTINTERFACE_H
