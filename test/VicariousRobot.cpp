#include "VicariousRobot.h"

VicariousRobot::VicariousRobot(std::map<std::string, std::shared_ptr<RBLink>> RBMap) {
    this->RBMap = RBMap;

    std::vector<std::string> prefixes = {"left_", "right_"};

    externalArm = Kine("External Arm");
    std::set<std::string> externalTreeEndConditions = {"left_arm_link_1", "right_arm_link_1", "camera_link_1"};
    externalArm.treeDiscovery(RBMap, "base_link_1", externalTreeEndConditions, prefixes);
    externalArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(externalArm));

    cameraArm = Kine("Camera");
    std::set<std::string> emptyEndConditions = {""};
    cameraArm.treeDiscovery(RBMap, "camera_link_1", emptyEndConditions, prefixes);
    cameraArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(cameraArm));

    std::map<std::string,double> armOffsets = {{"arm_link_2", EIGEN_PI}};
    leftArm = Kine("Left Arm");
    std::set<std::string> leftEndConditions = {"left_arm_link_8a", "left_arm_link_8b"};
    leftArm.treeDiscovery(RBMap, "left_arm_link_1", leftEndConditions, prefixes);
    leftArm.setJointsOffset(armOffsets);
    leftArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(leftArm));

    // Right Arm
    rightArm = Kine("Right Arm");
    std::set<std::string> rightEndConditions = {"right_arm_link_8a", "right_arm_link_8b"};
    rightArm.treeDiscovery(RBMap, "right_arm_link_1", rightEndConditions, prefixes);
    rightArm.setJointsOffset(armOffsets);
    rightArm.fkSolver = std::make_shared<FKSolverInterface>(FKSolverInterface(rightArm));

    std::vector<double> externalJoints = {0.0, 0.0, 0.0, 0.0, 0.0};
    externalArm.fk(externalJoints);

    std::vector<double> cameraJoints = {0.100, 0.0, 0.0, 0.0};
    cameraArm.fk(cameraJoints);

    // Elbow Plane Constraint Initialization
    auto Trot = Utils::eulXYZ2HMat(-EIGEN_PI/2.0, 0.0, 0.0);
    Eigen::Matrix4d Transl = Eigen::Matrix4d::Identity();
    Transl(2,3) = -0.02;
    Transl(0,3) = 0.100;
    elbowPlane = RBMap["base_link_5"]->absTransform * Trot * Transl;

//    std::cout << "This is elbow plane: " << std::endl;
//    std::cout << elbowPlane << std::endl;

//    std::vector<double> leftJoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> leftJoints = {0.100, (3.0/8.0)*EIGEN_PI, 0.5*EIGEN_PI, EIGEN_PI/6.0, (3.5/5.0)*EIGEN_PI, -EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
    leftArm.fk(leftJoints);

//    std::vector<double> rightJoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> rightJoints = {0.100, -(3.0/8.0)*EIGEN_PI, 0.5*EIGEN_PI, -EIGEN_PI/6.0, (3.5/5.0)*EIGEN_PI, -EIGEN_PI/3.0, EIGEN_PI/12.0, 0.0};
//    std::vector<double> rightJoints = {0.100, -(3.0/8.0)*EIGEN_PI, 0.5*EIGEN_PI, -EIGEN_PI/6.0, (3.5/5.0)*EIGEN_PI, -EIGEN_PI/3.0, EIGEN_PI/6.0, EIGEN_PI/6.0};
    rightArm.fk(rightJoints);

//    printKine(robot.externalArm);
//    printKine(robot.cameraArm);
//    printKine(robot.leftArm);
//    printKine(robot.rightArm);

    // IK

    leftArm.ikSolver = std::make_shared<Vicarious::ArmIKSolver>(Vicarious::ArmIKSolver(leftArm, elbowPlane));
    rightArm.ikSolver = std::make_shared<Vicarious::ArmIKSolver>(Vicarious::ArmIKSolver(rightArm, elbowPlane));

//    Eigen::Matrix4d left_target_transform = RBMap["left_arm_link_end_effector"]->absTransform;

//    std::cout << "Left target transform:" << std::endl;
//    std::cout << left_target_transform << std::endl;
//
//    Eigen::Matrix4d right_target_transform = RBMap["right_arm_link_end_effector"]->absTransform;
//    std::cout << "Right target transform:" << std::endl;
//    std::cout << right_target_transform << std::endl;
//
//    auto jointsLeft = leftArm.ik(left_target_transform);
//    auto jointsRight = rightArm.ik(right_target_transform);

//    std::cout << "Left arm joints: " << std::endl;
//    for(auto el : jointsLeft) std::cout << el << " ";
//    std::cout << std::endl;

//    std::cout << "Right arm joints: " << std::endl;
//    for(auto el : jointsRight) std::cout << el << " ";
//    std::cout << std::endl;

}
