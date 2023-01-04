#include "FKSolverInterface.h"

void FKSolverInterface::solve(const std::vector<double> joints) {
    if (chain->numJoints != joints.size()) std::cout << "The number of joints does not match requirements" << std::endl;

    auto rbLinkNow = chain->rootLink;

    int jointNum = -1; // initialize to reach 0th index after first increment
    while (chain->treeMap.count(rbLinkNow->name)) {
        auto joint = rbLinkNow->link->parent_joint;

        Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
        mat4.block(0, 3, 3, 1) = rbLinkNow->t;
        auto q = rbLinkNow->q;

        if (joint->type == urdf::JointType::REVOLUTE) {
            ++jointNum;
            auto axisSign = joint->axis.z;
            Eigen::Vector3d rotation(0, 0, (joints[jointNum] + rbLinkNow->jState.offset) * axisSign);
            rbLinkNow->jState.pos = joints[jointNum];
            double angle = rotation.norm();
            Eigen::Vector3d axis = rotation.normalized();
            Eigen::Quaterniond q_rot(Eigen::AngleAxisd(angle, axis));
            q = q * q_rot;
        }

        // Common operation for all joints including urdf::JointType::FIXED
        Eigen::Matrix3d mat3 = q.toRotationMatrix();
        mat4.block(0, 0, 3, 3) = mat3;

        if (joint->type == urdf::JointType::PRISMATIC) {
            ++jointNum;
            auto axisSign = joint->axis.z;
            mat4.block(0, 3, 3, 1) += mat4.block(0, 2, 3, 1) * (joints[jointNum] + rbLinkNow->jState.offset) * axisSign;
            rbLinkNow->jState.pos = joints[jointNum];
        }

        rbLinkNow->localTransform = mat4;
        rbLinkNow->absTransform = rbLinkNow->parent_link->absTransform * mat4;

        if (rbLinkNow->child_links.empty()) return;
        rbLinkNow = rbLinkNow->child_links[0];
    }
}

void FKSolverInterface::solveLinksRange(std::string startLinkGenericName, std::string endLinkGenericName, std::vector<double> joints) {
    if(!chain->genericTreeMap.count(startLinkGenericName) || !chain->genericTreeMap.count(endLinkGenericName)){
        std::cout << "Problem with solveLinksRange(): at least one of the names does not exist." << std::endl;
    }

    auto rbLinkNow = chain->genericTreeMap[startLinkGenericName];

    int jointNum = -1; // initialize to reach 0th index after first increment
    while (chain->treeMap.count(rbLinkNow->name)) {
        auto joint = rbLinkNow->link->parent_joint;

        Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
        mat4.block(0, 3, 3, 1) = rbLinkNow->t;
        auto q = rbLinkNow->q;

        if (joint->type == urdf::JointType::REVOLUTE) {
            ++jointNum;

            if(jointNum >= joints.size()) {
                std::cout << "Problem with joints Size in solveLinksRange." << std::endl;
            }

            auto axisSign = joint->axis.z;
            Eigen::Vector3d rotation(0, 0, (joints[jointNum] + rbLinkNow->jState.offset) * axisSign);
            rbLinkNow->jState.pos = joints[jointNum];
            double angle = rotation.norm();
            Eigen::Vector3d axis = rotation.normalized();
            Eigen::Quaterniond q_rot(Eigen::AngleAxisd(angle, axis));
            q = q * q_rot;
        }

        // Common operation for all joints including urdf::JointType::FIXED
        Eigen::Matrix3d mat3 = q.toRotationMatrix();
        mat4.block(0, 0, 3, 3) = mat3;

        if (joint->type == urdf::JointType::PRISMATIC) {
            ++jointNum;

            if(jointNum >= joints.size()) {
                std::cout << "Problem with joints Size in solveLinksRange." << std::endl;
            }

            auto axisSign = joint->axis.z;
            mat4.block(0, 3, 3, 1) += mat4.block(0, 2, 3, 1) * (joints[jointNum] + rbLinkNow->jState.offset) * axisSign;
            rbLinkNow->jState.pos = joints[jointNum];
        }

        rbLinkNow->localTransform = mat4;
        rbLinkNow->absTransform = rbLinkNow->parent_link->absTransform * mat4;

        if(rbLinkNow->genericName == endLinkGenericName) return;

        if (rbLinkNow->child_links.empty()) return;
        rbLinkNow = rbLinkNow->child_links[0];
    }

}

void FKSolverInterface::updateLinksWithChildren(std::string startLinkGenericName) {
    if(!chain->genericTreeMap.count(startLinkGenericName)){
        std::cout << "Problem with updateLinksWithChildren(): parent name does not exist." << std::endl;
    }

    auto rbLinkNow = chain->genericTreeMap[startLinkGenericName];

    std::queue<shared_ptr<RBLink>> queue;
    queue.push(rbLinkNow);

    while (!queue.empty()) {
        rbLinkNow = queue.front();
        queue.pop();

        auto joint = rbLinkNow->link->parent_joint;

        Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
        mat4.block(0, 3, 3, 1) = rbLinkNow->t;
        auto q = rbLinkNow->q;

        if (joint->type == urdf::JointType::REVOLUTE) {
            auto axisSign = joint->axis.z;
            Eigen::Vector3d rotation(0, 0, (rbLinkNow->jState.pos + rbLinkNow->jState.offset) * axisSign);
            double angle = rotation.norm();
            Eigen::Vector3d axis = rotation.normalized();
            Eigen::Quaterniond q_rot(Eigen::AngleAxisd(angle, axis));
            q = q * q_rot;
        }

        // Common operation for all joints including urdf::JointType::FIXED
        Eigen::Matrix3d mat3 = q.toRotationMatrix();
        mat4.block(0, 0, 3, 3) = mat3;

        if (joint->type == urdf::JointType::PRISMATIC) {
            auto axisSign = joint->axis.z;
            mat4.block(0, 3, 3, 1) += mat4.block(0, 2, 3, 1) * (rbLinkNow->jState.pos + rbLinkNow->jState.offset) * axisSign;
        }

        rbLinkNow->localTransform = mat4;
        rbLinkNow->absTransform = rbLinkNow->parent_link->absTransform * mat4;

        if (rbLinkNow->child_links.empty()) continue;
        for(auto el : rbLinkNow->child_links) queue.push(el);
    }

}
