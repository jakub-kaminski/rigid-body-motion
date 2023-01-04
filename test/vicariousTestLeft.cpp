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

void mypr(std::string s, double val){
    std::cout <<"Value " << s << ": ";
    std::cout << val << std::endl;
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
//    Eigen::Matrix4d right_target_transform = RBMap["right_arm_link_end_effector"]->absTransform;
    std::cout << "Left target transform:" << std::endl;
    std::cout << left_target_transform << std::endl;

//    auto jointsLeft = robot.leftArm.ikSolver->solve(left_target_transform);

    auto jointsLeft0 = robot.leftArm.ikSolver->solve(left_target_transform);
    jointsLeft0[1] -= EIGEN_PI;
    robot.leftArm.fk(jointsLeft0);
    Eigen::Matrix4d left_target_transform0 = RBMap["left_arm_link_end_effector"]->absTransform;

    // Joints before
    Eigen::Matrix4d link_arm_3_before = RBMap["left_arm_link_3"]->absTransform;
    Eigen::Matrix4d link_arm_4_before = RBMap["left_arm_link_4"]->absTransform;
    Eigen::Matrix4d link_arm_5_before = RBMap["left_arm_link_5"]->absTransform;
    Eigen::Matrix4d link_arm_6_before = RBMap["left_arm_link_6"]->absTransform;
    Eigen::Matrix4d link_arm_7_before = RBMap["left_arm_link_7"]->absTransform;
    Eigen::Matrix4d link_arm_8_before = RBMap["left_arm_link_8"]->absTransform;
    Eigen::Matrix4d link_arm_end_effector_before = RBMap["left_arm_link_end_effector"]->absTransform;

    auto jointsLeft1 = robot.leftArm.ikSolver->solve(left_target_transform0);
    jointsLeft1[1] -= EIGEN_PI;
    robot.leftArm.fk(jointsLeft1);

    // Joints after
    Eigen::Matrix4d link_arm_3_after = RBMap["left_arm_link_3"]->absTransform;
    Eigen::Matrix4d link_arm_4_after = RBMap["left_arm_link_4"]->absTransform;
    Eigen::Matrix4d link_arm_5_after = RBMap["left_arm_link_5"]->absTransform;
    Eigen::Matrix4d link_arm_6_after = RBMap["left_arm_link_6"]->absTransform;
    Eigen::Matrix4d link_arm_7_after = RBMap["left_arm_link_7"]->absTransform;
    Eigen::Matrix4d link_arm_8_after = RBMap["left_arm_link_8"]->absTransform;
    Eigen::Matrix4d link_arm_end_effector_after = RBMap["left_arm_link_end_effector"]->absTransform;

    Eigen::Vector3d upperArmLenVec_after = link_arm_3_after.block(0,3,3,1) - link_arm_5_after.block(0,3,3,1);
    double upperArmLen_after = upperArmLenVec_after.norm();

    Eigen::Vector3d lowerArmLenVec_after = link_arm_7_after.block(0,3,3,1) - link_arm_5_after.block(0,3,3,1);
    double lowerArmLen_after = lowerArmLenVec_after.norm();

    Eigen::Vector3d wristLenVec_after = link_arm_8_after.block(0,3,3,1) - link_arm_7_after.block(0,3,3,1);
    double wristLen_after = wristLenVec_after.norm();



    //Diff Mat
    Eigen::Matrix4d link_arm_3_diff = link_arm_3_after - link_arm_3_before;
    Eigen::Matrix4d link_arm_4_diff = link_arm_4_after - link_arm_4_before;
    Eigen::Matrix4d link_arm_5_diff = link_arm_5_after - link_arm_5_before;
    Eigen::Matrix4d link_arm_6_diff = link_arm_6_after - link_arm_6_before;
    Eigen::Matrix4d link_arm_7_diff = link_arm_7_after - link_arm_7_before;
    Eigen::Matrix4d link_arm_8_diff = link_arm_8_after - link_arm_8_before;
    Eigen::Matrix4d link_arm_end_effector_diff = link_arm_end_effector_after - link_arm_end_effector_before;
    mypr("Link Arm 3 Diff", link_arm_3_diff);
    mypr("Link Arm 4 Diff", link_arm_4_diff);
    mypr("Link Arm 5 Diff", link_arm_5_diff);
    mypr("Link Arm 6 Diff", link_arm_6_diff);
    mypr("Link Arm 7 Diff", link_arm_7_diff);
    mypr("Link Arm 8 Diff", link_arm_8_diff);
    mypr("Link Arm End Effector Diff", link_arm_end_effector_diff);

    std::cout << "Joint Diffs: " << std::endl;
    for(int i = 0; i < jointsLeft0.size(); ++i){
        std::cout << "J" << i+1 << " diff: " << jointsLeft1[i]-jointsLeft0[i] << std::endl;
    }

    mypr("UpperArmLen_after", upperArmLen_after);
    mypr("lowerArmLen_after", lowerArmLen_after);
    mypr("wristLen_after", wristLen_after);

    std::cout << "Main Loop Copmpute" << std::endl;
//    std::cout << "Left arm joints: " << std::endl;
//    for(auto el : jointsLeft) std::cout << el << " ";
//    std::cout << std::endl;

//    std::cout << "Right arm joints: " << std::endl;
//    for(auto el : jointsRight0) std::cout << el << " ";
//    std::cout << std::endl;
//
    Eigen::Matrix4d left_target_transform_after = RBMap["left_arm_link_end_effector"]->absTransform;
//    std::cout << "Right target transform_after:" << std::endl;
//    std::cout << right_target_transform_after << std::endl;

//    auto jointsRight = robot.rightArm.ik(right_target_transform);


    return 0;
}
