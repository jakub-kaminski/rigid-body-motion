#include <iostream>
#include <chrono>
#include <unistd.h>

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

//    Eigen::Matrix4d left_target_transform = RBMap["left_arm_link_end_effector"]->absTransform;
    Eigen::Matrix4d right_target_transform = RBMap["right_arm_link_end_effector"]->absTransform;

    Eigen::Matrix4d right_target_transform_original = right_target_transform;

    std::cout << "Right target transform:" << std::endl;
    std::cout << right_target_transform << std::endl;

//    auto jointsLeft = robot.leftArm.ikSolver->solve(left_target_transform);

    auto start = chrono::steady_clock::now();
    int iterations = 100000;
    for(int i = 0; i < iterations; ++i){
        right_target_transform(1,3) += 0.0002;
        auto jointsRight0 = robot.rightArm.ikSolver->solve(right_target_transform);
        jointsRight0[1] -= EIGEN_PI;
        robot.rightArm.fk(jointsRight0);
        Eigen::Matrix4d right_target_transform0 = RBMap["right_arm_link_end_effector"]->absTransform;
    }
    auto end = chrono::steady_clock::now();
    cout << "Elapsed time in milliseconds: " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;

    cout << "Elapsed time in microseconds: "
    << chrono::duration_cast<chrono::microseconds>(end - start).count()
    << " µs" << endl;

    cout << "Elapsed time in nanoseconds: "
    << chrono::duration_cast<chrono::nanoseconds>(end - start).count()
    << " ns" << endl;

    cout << "Nanoseconds per iteration: "
    << chrono::duration_cast<chrono::nanoseconds>(end - start).count() / double( iterations )
    << " ns" << endl;


    return 0;
}