#include "RobotInterface.h"

std::map<std::string, std::vector<double>> RobotInterface::getSerializedTransforms() {
    // Bind names to raw matrix data
    for(auto& item : RBMap){
        serializedMap[item.first] = Utils::matrixToVector(item.second->absTransform);
    }
    return serializedMap;
}

RobotInterface::RobotInterface(std::map<std::string, std::shared_ptr<RBLink>> RBMap, Eigen::Matrix4d adjust) {
    this->RBMap = RBMap;
    this->adjust = adjust;
}

std::vector<double> RobotInterface::getTransformByID(std::string id) {
    if(!RBMap.count(id)) std::cout << "ID " << id << " not found." << std::endl;
    return getVectorFromMatrix(RBMap[id]->absTransform);
}

void RobotInterface::setAdjustTransform(Eigen::Matrix4d mat) {
    this->adjust = mat;
}

// Note that the output transform is rotated by the adjust matrix
// To accomodate for default ThreeJS world orientation (Y-axis up)
// The transform is serialized to a vector
std::vector<double> RobotInterface::getVectorFromMatrix(Eigen::Matrix4d mat) {
    std::vector<double> res;
    Eigen::Matrix4d matAdjusted = adjust * mat;

    res.reserve(16);
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            res.push_back(matAdjusted(r, c)*1.0);
        }
    }
    return res;
}
