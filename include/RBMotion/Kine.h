#pragma once

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <set>

#include <Eigen/Dense>
#include <Eigen/Core>

#include "urdf/model.h"
#include "RBLink.h"
#include "FKSolverInterface.h"
#include "IKSolverInterface.h"
#include "Utils.h"

class FKSolverInterface;
class IKSolverInterface;

class Kine {
public:
    std::string chainName;
    int numJoints = 0;
    shared_ptr<RBLink> rootLink;
    shared_ptr<RBLink> parentLink = nullptr;
    std::map<std::string, shared_ptr<RBLink>> treeMap;
    std::map<std::string, shared_ptr<RBLink>> genericTreeMap;
    std::shared_ptr<FKSolverInterface> fkSolver;
    std::shared_ptr<IKSolverInterface> ikSolver;

    Kine() = default;
    Kine(std::string name) : chainName(name) {}

    void treeDiscovery(std::map<std::string, shared_ptr<RBLink>> &RBMap, std::string rootName,
                       const std::set<std::string>& excludedEndLinks, const std::vector<std::string>& prefixes);

    Eigen::Matrix4d relativeTransform(std::string childLinkName, std::string baseLinkName);

    void setJointsOffset(std::map<std::string, double> &jointOffsets);


    void fk(std::vector<double> joints);

    std::vector<double> ik(Eigen::Matrix4d endEffectorTransform);
};