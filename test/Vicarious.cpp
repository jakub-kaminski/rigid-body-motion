#include "Vicarious.h"
#include "../../../include/Vicarious.h"


void pr(std::string s, Eigen::Matrix4d T){
    std::cout <<"Matrix " << s << ": " << std::endl;
    std::cout << T << std::endl;
}

void pr(std::string s, Eigen::Vector3d v){
    std::cout <<"Vector " << s << ": " << std::endl;
    std::cout << v << std::endl;
}

void pr(std::string s, double d){
    std::cout <<"Value " << s << ": " << std::endl;
    std::cout << d << std::endl;
}

double Vicarious::ArmIKSolver::elbowIKSolve(const Eigen::Matrix4d& endEff, TransientData &data, double joint7) {
    double img_wrist_r =  lowArmLength * cos(joint7) + wristLinkLength;
    double transl_wrist = lowArmLength * sin(joint7);

    Eigen::Vector3d C = Utils::vectorProjection(data.sphere_center-data.wrist_pos, -data.wrist_axis_z);
    Eigen::Vector3d unit_C = C / C.norm();
    Eigen::Vector3d wrist_transl_C = unit_C * transl_wrist;
    double len_C = C.norm();
    double cone_h = len_C - transl_wrist;
    double img_sphere_r = sqrt(pow(upArmLength,2)- pow(cone_h, 2));

    Eigen::Matrix4d img_wristT = data.wristT;
    img_wristT.block(0,3,3,1) += wrist_transl_C;

//    std::cout << "******************" << std::endl;
//    pr("data.sphere_center", data.sphere_center);
//    pr("data.wrist_pos", data.wrist_pos);
//
//    pr("C", C);
//    pr("unit_C", unit_C);
//    pr("wrist_transl_C", wrist_transl_C);
//    pr("len_C", len_C);
//    pr("cone_h", cone_h);
//    pr("img_sphere_r", img_sphere_r);
//    pr("img_wristT", img_wristT);

    auto pt_out = Utils::my3Dcirc(data.wrist_pos, img_wrist_r, data.sphere_center, img_sphere_r, img_wristT);
//    pr("pt_out.first", pt_out.first);
//    pr("pt_out.second", pt_out.second);

    double val1 = (pt_out.first - data.lastElbowPos).norm();
    double val2 = (pt_out.second - data.lastElbowPos).norm();

    if( (pt_out.first - data.lastElbowPos).norm() < (pt_out.second - data.lastElbowPos).norm()){
        data.candidateElbowPos = pt_out.first;
    }
    else{
        data.candidateElbowPos = pt_out.second;
    }

//    pr("Elbow Target Plane: ", *elbowPlane);

    double err = Utils::pointToPlaneSignedDistance(data.candidateElbowPos, *elbowPlane);
    return err;
}


std::vector<double> Vicarious::ArmIKSolver::solve(const Eigen::Matrix4d endEff) {
   Eigen::Matrix4d sphereT = chain->genericTreeMap["arm_link_3"]->absTransform;

    TransientData data;
    data.sphere_center = sphereT.block(0,3,3,1);
    data.lastElbowPos = chain->genericTreeMap["arm_link_5"]->absTransform.block(0,3,3,1);

    data.wristT = endEff * effectorLocalTransform.inverse();
    data.wrist_pos = data.wristT.block(0,3,3,1);
    data.wrist_axis_z = data.wristT.block(0,2,3,1);

    double Wrist7JointPos = chain->genericTreeMap["arm_link_7"]->jState.pos;

//    pr("wristT", data.wristT);
//    pr("wrist_pos", data.wrist_pos);
//    pr("wrist_axis_z", data.wrist_axis_z);
//    pr("sphere_center", data.sphere_center);
//    pr("Wrist7JointPos", Wrist7JointPos);


    for(int i = 0; i < 6; ++i){
        double err1 = elbowIKSolve(endEff, data, Wrist7JointPos);
//        std::cout << "Current Error is: " << err1 << std::endl;
        Wrist7JointPos += d_angle;
        double err2 = elbowIKSolve(endEff, data, Wrist7JointPos);
        Wrist7JointPos -= d_angle;

        double adjustment = Utils::calculateAdjustment(err1, err2, d_angle);
        Wrist7JointPos += adjustment;
//        std::cout << "Eff" << std::endl;
//        std::cout << endEff << std::endl;
//        std::cout << data.candidateElbowPos << std::endl;
//        std::cout << "Last Angle: " << Wrist7JointPos << std::endl;
    }
//    std::cout << "Last Elbow Pos: " << std::endl;
//    std::cout << data.lastElbowPos << std::endl;
//    std::cout << "Candidate Elbow Pos: " << std::endl;
//    std::cout << data.candidateElbowPos << std::endl;
   data.lastElbowPos = data.candidateElbowPos;

    double Linear1JointPos = chain->genericTreeMap["arm_link_1"]->jState.pos;
    std::vector<double> joints = {Linear1JointPos, 0.0, 0.0, 0.0, 0.0, 0.0, Wrist7JointPos, 0.0};

    Eigen::Vector3d tmpVector = data.lastElbowPos - data.wrist_pos;
    Eigen::Vector3d up_wrist_transl = data.wrist_axis_z.cross(tmpVector).cross(data.wrist_axis_z);
    up_wrist_transl = wristLinkLength * up_wrist_transl / up_wrist_transl.norm();
    Eigen::Vector3d up_wrist_pos = data.wrist_pos + up_wrist_transl;

    Eigen::Matrix4d pt_out_frame = data.wristT;
    pt_out_frame.block(0,3,3,1) = up_wrist_pos;

    Eigen::Vector3d up_arm_vec = data.lastElbowPos - data.sphere_center;
    Eigen::Vector3d low_arm_vec = up_wrist_pos - data.lastElbowPos;

    Eigen::Matrix4d j1_parent_T = chain->genericTreeMap["arm_link_1fixed"]->absTransform;
    Eigen::Vector3d up_arm_proj1 = Utils::ortoproj(j1_parent_T.block(0,0,3,1), j1_parent_T.block(0,1,3,1), up_arm_vec);

    joints[1] = Utils::goodvecangle(j1_parent_T.block(0,1,3,1), up_arm_proj1, j1_parent_T.block(0,2,3,1));

    Eigen::Vector3d a3 = up_arm_vec.cross(Eigen::Vector3d(j1_parent_T.block(0,2,3,1)));
    joints[2] = Utils::goodvecangle(up_arm_vec, j1_parent_T.block(0,2,3,1), a3);

    // Note the offset
    chain->fkSolver->solveLinksRange("arm_link_2", "arm_link_4", {joints[1] - M_PI, joints[2], 0.0});
    Eigen::Matrix4d raw_frame_j4 = chain->genericTreeMap["arm_link_4"]->absTransform;

//    pr("raw_frame_j4", raw_frame_j4);

    Eigen::Vector3d low_arm_vec_proj3 = Utils::ortoproj(raw_frame_j4.block(0,0,3,1), raw_frame_j4.block(0,1,3,1), low_arm_vec);

//    pr("low_arm_vec_proj3", low_arm_vec_proj3);

    joints[3] = Utils::goodvecangle(raw_frame_j4.block(0,1,3,1), low_arm_vec_proj3, raw_frame_j4.block(0,2,3,1));
    Eigen::Vector3d a5 = up_arm_vec.cross(low_arm_vec);

//    pr("a5", a5);

    joints[4] = Utils::goodvecangle(up_arm_vec, low_arm_vec, a5);
    chain->fkSolver->solveLinksRange("arm_link_4", "arm_link_7", {joints[3], joints[4], 0.0, 0.0});

    Eigen::Matrix4d raw_frame_j6 = chain->genericTreeMap["arm_link_6"]->absTransform;

    Eigen::Vector3d wrist_axis_z_proj6 = Utils::ortoproj(raw_frame_j6.block(0,0,3,1), raw_frame_j6.block(0,1,3,1), data.wrist_axis_z);
    joints[5] = Utils::goodvecangle(raw_frame_j6.block(0,1,3,1), -wrist_axis_z_proj6, raw_frame_j6.block(0,2,3,1));
    chain->fkSolver->solve(joints);

//    joints[7] = Utils::goodvecangle(up_wrist_transl, data.wristT.block(0,3,3,1) - endEff.block(0,3,3,1), endEff.block(0,1,3,1));


    Eigen::Matrix4d frame_j7 = chain->genericTreeMap["arm_link_7"]->absTransform;
    double joint6Candidate = Utils::goodvecangle(up_wrist_transl, -low_arm_vec, frame_j7.block(0,2,3,1));
    joints[6] = joint6Candidate;

    joints[7] = Utils::goodvecangle(up_wrist_transl, -endEff.block(0,2,3,1), endEff.block(0,1,3,1));
//    Eigen::Vector3d newWristCenter = endEff.block(0,3,3,1) - 0.0145 * endEff.block(0,2,3,1);
//    std::cout << "***** new wrist center ******" << std::endl;
//    std::cout << newWristCenter << std::endl;
//    std::cout << "***** wrist pos ******" << std::endl;
//    std::cout << data.wrist_pos << std::endl;
//    std::cout << "***** wrist T ******" << std::endl;
//    std::cout << data.wristT << std::endl;

    chain->fkSolver->solveLinksRange("arm_link_6", "arm_link_end_effector", {joints[5], joints[6], joints[7]});

//    std::cout << "Hello" << std::endl;
//    double newJoint7Val = Wrist7JointPos;

    return joints;

//    Eigen::Vector3d elbowCenter = elbowTransform.block(0,3,3,1);
}

//void Vicarious::ArmIKSolver::initializeConstants() {
//}

//Eigen::Vector3d Vicarious::ArmIKSolver::computeElbowCenter(void) {
//    return Eigen::Vector3d();
//}
