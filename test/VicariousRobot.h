#ifndef MYLIBRARYPROJECT_VICARIOUSROBOT_H
#define MYLIBRARYPROJECT_VICARIOUSROBOT_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Kine.h>
#include "RobotInterface.h"

#include "Vicarious.h"

class VicariousRobot : public RobotInterface {
public:
    Kine externalArm;
    Kine cameraArm;
    Kine leftArm;
    Kine rightArm;

    Eigen::Matrix4d elbowPlane;

    explicit VicariousRobot(std::map<std::string, std::shared_ptr<RBLink>> RBMap);

};


#endif //MYLIBRARYPROJECT_VICARIOUSROBOT_H
