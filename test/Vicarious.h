#ifndef MYLIBRARYPROJECT_VICARIOUS_H
#define MYLIBRARYPROJECT_VICARIOUS_H

#include <Kine.h>

namespace Vicarious
{
//    class CameraIKSolver : public IKSolverInterface{
//        std::shared_ptr<Kine> kine;
//    public:
//        explicit CameraIKSolver(Kine &chain) : IKSolverInterface(chain) {
//            this->kine = std::make_shared<Kine>(chain);
//        }
//
//        void solve(Eigen::Matrix4d endEff) override;
//    };
    struct TransientData{
        Eigen::Vector3d sphere_center;
        Eigen::Vector3d lastElbowPos;
        Eigen::Vector3d candidateElbowPos;
        Eigen::Matrix4d wristT;
        Eigen::Vector3d wrist_pos;
        Eigen::Vector3d wrist_axis_z;
    };

    class ArmIKSolver : public IKSolverInterface{
        double upArmLength;
        double lowArmLength;
        double wristLinkLength;
        double d_angle = EIGEN_PI/1080.0;
//        Eigen::Vector3d lastWristPos;
        Eigen::Matrix4d effectorLocalTransform;
        std::shared_ptr<Kine> kine;
        std::shared_ptr<Eigen::Matrix4d> elbowPlane;

        double elbowIKSolve(const Eigen::Matrix4d& endEff, TransientData &data, double joint7);

    public:
        explicit ArmIKSolver(Kine &chain, Eigen::Matrix4d &elbowPlane) : IKSolverInterface(chain) {
//            kine = std::make_shared<Kine>(chain);
            this->elbowPlane = std::make_shared<Eigen::Matrix4d>(elbowPlane);

            auto upArmTransform = this->chain->relativeTransform("arm_link_5", "arm_link_3");
            auto lowArmTransform = this->chain->relativeTransform("arm_link_7", "arm_link_5");
            auto wristLinkTransform = this->chain->genericTreeMap["arm_link_8"]->localTransform;
            auto tmp = this->chain->genericTreeMap["arm_link_7"]->localTransform;
            upArmLength = upArmTransform(0, 3);
            lowArmLength = lowArmTransform(0, 3);
            wristLinkLength = wristLinkTransform(0,3);
            this->effectorLocalTransform = this->chain->genericTreeMap["arm_link_end_effector"]->localTransform;
        }

        std::vector<double> solve(Eigen::Matrix4d endEff) override;
    };
}

#endif //MYLIBRARYPROJECT_VICARIOUS_H
