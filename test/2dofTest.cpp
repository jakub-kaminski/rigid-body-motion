#include <Kine.h>
#include "2dofURDF.h"
#include "vicariousURDF.h"
//vicarious

#include <string>
#include "urdf/model.h"

// localHMat(std::shared_ptr<urdf::Joint> joint){
Eigen::Matrix4d localHMat(std::shared_ptr<urdf::Joint> joint, double radAngle) {

//    Eigen::Quaterniond q(w, x, y, z);
    Eigen::Quaterniond q(
            joint->parent_to_joint_transform.rotation.w,
            joint->parent_to_joint_transform.rotation.x,
            joint->parent_to_joint_transform.rotation.y,
            joint->parent_to_joint_transform.rotation.z
    );
//    q.w(), q.vec()

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

std::vector<std::shared_ptr<urdf::Link>> serialDiscovery(std::shared_ptr<urdf::UrdfModel> model) {
    std:
    shared_ptr<urdf::Link> linkNow = model->root_link;
    std::vector<std::shared_ptr<urdf::Link>> links;

//    if(linkNow->child_links.size() == 1) linkNow = linkNow->child_links[0];

    while (true) {
        links.push_back(linkNow);
        if (linkNow->child_links.empty()) break;
        linkNow = linkNow->child_links[0];
    }

    return links;
}

vector<Eigen::Matrix4d>
serialFK(const Eigen::Matrix4d baseTransform, std::vector<std::shared_ptr<urdf::Link>> links, vector<double> angles) {
    vector<Eigen::Matrix4d> res;
    res.reserve(links.size());
    res.push_back(baseTransform);

    for (int i = 1; i < links.size(); ++i) {
        double angle = 0.0;
        if (i <= angles.size()) angle = angles[i - 1];
        Eigen::Matrix4d localTransform = localHMat(links[i]->parent_joint, angle);
        Eigen::Matrix4d absTransform = res.back() * localTransform;
        res.push_back(absTransform);
    }
    return res;
}

int main() {

    std::shared_ptr<urdf::UrdfModel> model;
//    model = urdf::UrdfModel::fromUrdfStr(std::string(urdfstr_2dof));
    model = urdf::UrdfModel::fromUrdfStr(std::string(vicarious));

    auto root = model->getRoot();

    auto links = serialDiscovery(model);
    Eigen::Matrix4d baseMat = Eigen::Matrix4d::Identity();
    std::vector<double> angles = {-EIGEN_PI / 4, EIGEN_PI / 4};
    auto mats = serialFK(baseMat, links, angles);

    std::cout << "Auto Discovery Transformations: " << std::endl;
    for(int i = 1; i < mats.size(); ++i){
        if(i == 1 || i == 2) std::cout << "Link " << i << std::endl;
        if(i == 3) std::cout << "EndEffector" << std::endl;
        std::cout << mats[i] << std::endl;
    }
//    std::cout << mats.back() << std::endl;




    auto link1 = model->getLink("link_1");
    auto joint1 = model->getJoint("joint_1");
    auto joint2 = model->getJoint("joint_2");
    auto end_effector_joint = model->getJoint("end_effector_joint");



//    if(joint1->type == urdf::JointType::REVOLUTE)


//    auto tm = model->
    double radAngle1 = - EIGEN_PI / 4;
    Eigen::Matrix4d m1 = localHMat(joint1, radAngle1);
    double radAngle2 = EIGEN_PI / 4;
    Eigen::Matrix4d m2 = localHMat(joint2, radAngle2);
    Eigen::Matrix4d m3 = localHMat(end_effector_joint, 0);

    Eigen::Matrix4d res = m1 * m2 * m3;
    std::cout << "End Eff Matrix: " << std::endl;
    std::cout << res << std::endl;

//    localHMat(joint2);
//    localHMat(end_effector_joint);

    std::cout << "Hi there!" << std::endl;

    return 0;
}