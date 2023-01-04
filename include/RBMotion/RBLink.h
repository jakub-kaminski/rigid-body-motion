//
// Created by jakub on 12/6/22.
//

#ifndef MYLIBRARYPROJECT_RBLINK_H
#define MYLIBRARYPROJECT_RBLINK_H

#include <Eigen/Dense>
#include <Eigen/Core>

#include <memory>
#include <vector>

#include <string>
#include "urdf/model.h"

struct JointState {
    double offset = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;
    double torque = 0.0;
    JointState() = default;
};

class RBLink {
public:
    std::string name;
    std::string genericName;
    std::shared_ptr<urdf::Link> link;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::Matrix4d localTransform;
    Eigen::Matrix4d absTransform = Eigen::Matrix4d::Identity();

    JointState jState = JointState();
    std::shared_ptr<RBLink> parent_link = nullptr;
    std::vector<std::shared_ptr<RBLink>> child_links;
    RBLink(std::shared_ptr<urdf::Link> l) : link{l} {}
};


#endif //MYLIBRARYPROJECT_RBLINK_H
