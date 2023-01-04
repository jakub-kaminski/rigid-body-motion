#include <Kine.h>
#include "2dofURDF.h"
#include "vicariousURDF.h"

#include "Vicarious.h"
#include "VicariousRobot.h"

//#include <string>
//#include "urdf/model.h"
//#include "RBLink.h"
#include "RBTreeParser.h"


void mypr(std::string s, Eigen::Matrix4d T){
    std::cout <<"Matrix " << s << ": " << std::endl;
    std::cout << T << std::endl;
}

void printKine(Kine kine){
    auto el = kine.rootLink;
    std::cout << "***** "<< kine.chainName << " *****" << std::endl;
    while(kine.treeMap.count(el->name)){
        std::cout << el->name << std::endl;
        std::cout << el->absTransform << std::endl;
        if(el->child_links.empty()) break;
        el = el->child_links[0];
    }
}

int main() {
    std::shared_ptr<urdf::UrdfModel> model;
    model = urdf::UrdfModel::fromUrdfStr(std::string(vicarious));
    std::map<std::string, std::shared_ptr<RBLink>> RBMap = parseToRBLinkMap(model);

    auto robot = VicariousRobot(RBMap);


//    Eigen::Matrix4d right_ = RBMap["left_arm_link_end_effector"]->absTransform;

    Eigen::Matrix4d left_target_transform = RBMap["left_arm_link_end_effector"]->absTransform;
    Eigen::Matrix4d right_target_transform = RBMap["right_arm_link_end_effector"]->absTransform;
    std::cout << "Right target transform:" << std::endl;
    std::cout << right_target_transform << std::endl;

//    auto jointsLeft = robot.leftArm.ikSolver->solve(left_target_transform);

    auto jointsRight0 = robot.rightArm.ikSolver->solve(right_target_transform);
    jointsRight0[1] -= EIGEN_PI;
    robot.rightArm.fk(jointsRight0);
    Eigen::Matrix4d right_target_transform0 = RBMap["right_arm_link_end_effector"]->absTransform;

    // Joints before
    Eigen::Matrix4d link_arm_5_before = RBMap["right_arm_link_5"]->absTransform;
    Eigen::Matrix4d link_arm_6_before = RBMap["right_arm_link_6"]->absTransform;
    Eigen::Matrix4d link_arm_7_before = RBMap["right_arm_link_7"]->absTransform;
    Eigen::Matrix4d link_arm_8_before = RBMap["right_arm_link_8"]->absTransform;
    Eigen::Matrix4d link_arm_end_effector_before = RBMap["right_arm_link_end_effector"]->absTransform;

    auto jointsRight1 = robot.rightArm.ikSolver->solve(right_target_transform0);
    jointsRight1[1] -= EIGEN_PI;
    robot.rightArm.fk(jointsRight1);

    // Joints after
    Eigen::Matrix4d link_arm_5_after = RBMap["right_arm_link_5"]->absTransform;
    Eigen::Matrix4d link_arm_6_after = RBMap["right_arm_link_6"]->absTransform;
    Eigen::Matrix4d link_arm_7_after = RBMap["right_arm_link_7"]->absTransform;
    Eigen::Matrix4d link_arm_8_after = RBMap["right_arm_link_8"]->absTransform;
    Eigen::Matrix4d link_arm_end_effector_after = RBMap["right_arm_link_end_effector"]->absTransform;

    //Diff Mat
    Eigen::Matrix4d link_arm_5_diff = link_arm_5_after - link_arm_5_before;
    Eigen::Matrix4d link_arm_6_diff = link_arm_6_after - link_arm_6_before;
    Eigen::Matrix4d link_arm_7_diff = link_arm_7_after - link_arm_7_before;
    Eigen::Matrix4d link_arm_8_diff = link_arm_8_after - link_arm_8_before;
    Eigen::Matrix4d link_arm_end_effector_diff = link_arm_end_effector_after - link_arm_end_effector_before;
    mypr("Link Arm 5 Diff", link_arm_6_diff);
    mypr("Link Arm 6 Diff", link_arm_6_diff);
    mypr("Link Arm 7 Diff", link_arm_7_diff);
    mypr("Link Arm 8 Diff", link_arm_8_diff);
    mypr("Link Arm End Effector Diff", link_arm_end_effector_diff);

    std::cout << "Joint Diffs: " << std::endl;
    for(int i = 0; i < jointsRight0.size(); ++i){
        std::cout << "J" << i+1 << " diff: " << jointsRight1[i]-jointsRight0[i] << std::endl;
    }


    std::cout << "Main Loop Copmpute" << std::endl;
//    std::cout << "Left arm joints: " << std::endl;
//    for(auto el : jointsLeft) std::cout << el << " ";
//    std::cout << std::endl;

//    std::cout << "Right arm joints: " << std::endl;
//    for(auto el : jointsRight0) std::cout << el << " ";
//    std::cout << std::endl;
//
    Eigen::Matrix4d right_target_transform_after = RBMap["right_arm_link_end_effector"]->absTransform;
//    std::cout << "Right target transform_after:" << std::endl;
//    std::cout << right_target_transform_after << std::endl;

//    auto jointsRight = robot.rightArm.ik(right_target_transform);


    return 0;
}