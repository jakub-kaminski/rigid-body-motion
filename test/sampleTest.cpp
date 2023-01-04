#include <Kine.h>

#include "urdf/model.h"
#include <string>
#include <iostream>
#include <exception>
#include "tinyxml/txml.h"
#include "unused.h"

#include "sampleURDF.h"

//Eigen::Quaterniond euler2Quaternion( const double roll, const double pitch, const double yaw )
//{
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
//
//    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
//    return q;
//}

//    Eigen::Quaterniond q2 = euler2Quaternion(0,0, M_PI_4);


//class VicariousArmSolver : public IKSolver{
//    // add joint structure
//    void solve(Eigen::Matrix4d, ) = 0;
//
//    // last elbow plane transform
//    // last elbow center pos
//};

int main(){
    std::shared_ptr<urdf::UrdfModel> model;
    model = urdf::UrdfModel::fromUrdfStr(std::string(urdfstr_two_segment));

    auto root = model->getRoot();

    auto link1 = model->getLink("link_1");

    auto inertial = &link1->inertial.value();

    std::vector<std::array<double,4>> dhArray = {
            {1.0, 1.0, 1.0, 1.0},
            {1.0, 1.0, 1.0, 1.0},
            {1.0, 1.0, 1.0, 1.0}
    };

    std::vector<bool> isDisplayedFrame = {
            true, false, true
    };

    Kine kin;

    kin.initFrames(dhArray, isDisplayedFrame);

    kin.printDisplayFrames();

    return 0;
}
