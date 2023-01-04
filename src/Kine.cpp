#include "Kine.h"

#include <queue>

//shared_ptr<urdf::Link> getLinkByID(const std::shared_ptr<urdf::UrdfModel>& model, std::string targetLinkName) {
//    shared_ptr<urdf::Link> linkNow = model->root_link;
//
//    std::queue<shared_ptr<urdf::Link>> q;
//    q.push(linkNow);
//
//    while (!q.empty()) {
//        auto item = q.front();
//        q.pop();
//        if (item->name == targetLinkName) return item;
//        for (auto link: item->child_links) q.push(link);
//    }
//
//    std::cout << "Error!" << std::endl;
//
//    return nullptr;
//}

void Kine::treeDiscovery(std::map<std::string, shared_ptr<RBLink>> &RBMap, std::string rootName,
                         const std::set<std::string>& excludedEndLinks, const std::vector<std::string>& prefixes) {
    if (RBMap[rootName] == nullptr) std::cout << "ERROR" << std::endl;
    std::queue<shared_ptr<RBLink>> q;

    this->rootLink = RBMap[rootName];
    if (this->rootLink->link->parent_link != nullptr) {
        this->parentLink = RBMap[this->rootLink->link->parent_link->name];
    }
    q.push(RBMap[rootName]);

    std::map<std::string, shared_ptr<RBLink>> map;
    std::map<std::string, shared_ptr<RBLink>> genericMap;

    while (!q.empty()) {
        auto item = q.front();
        q.pop();
        if (excludedEndLinks.count(item->name)) continue;

        if (item->link->parent_joint != nullptr && item->link->parent_joint->type != urdf::JointType::FIXED) {
            this->numJoints++;
        }

        if (map.count(item->name)) std::cout << "Issues in Tree Discovery " << std::endl;

        if (!excludedEndLinks.count(item->name)) {
            map[item->name] = item;

            bool found = false;
            for (const auto &prefix: prefixes) {
                std::string genericLinkName = item->name;
                // Find the starting position of substring in the string
                std::size_t ind = genericLinkName.find(prefix);
                if (ind != std::string::npos) {
                    found = true;
                    genericLinkName.erase(ind, prefix.length());
                    genericMap[genericLinkName] = item;
                    item->genericName = genericLinkName;
                }
            }

            if (!found) {
                genericMap[item->name] = item;
                item->genericName = item->name;
            }
            for (const auto &link: item->child_links) q.push(link);
        }
    }

    this->treeMap = map;
    this->genericTreeMap = genericMap;
}

void Kine::setJointsOffset(std::map<std::string, double> &jointOffsets) {
    for (auto &item: this->genericTreeMap) {
        if (jointOffsets.count(item.second->genericName)) {
            item.second->jState.offset = jointOffsets[item.second->genericName];
        }
    }
}

void Kine::fk(std::vector<double> joints) {
    this->fkSolver->solve(joints);
}

std::vector<double> Kine::ik(Eigen::Matrix4d endEffectorTransform) {
    return this->ikSolver->solve(endEffectorTransform);
}

Eigen::Matrix4d Kine::relativeTransform(std::string childLinkName, std::string baseLinkName) {
    if(!genericTreeMap.count(childLinkName) || !genericTreeMap.count(baseLinkName)) {
        std::cout << "Name mismatch in relative transform function" << std::endl;
    }
    auto childLinkTransform = genericTreeMap[childLinkName]->absTransform;
    auto baseLinkTransform = genericTreeMap[baseLinkName]->absTransform;

    auto child2BaseTransform = baseLinkTransform.inverse() * childLinkTransform;

    return child2BaseTransform;
}

//void Kine::fk(std::vector<double> joints) {
//
//    localHMat(links[i]->parent_joint, angle);
//}

// Utility functions
std::vector<double> getVectorFromMatrix(Eigen::Matrix4d mat) {
    std::vector<double> res;
    res.reserve(16);
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            res.push_back(mat(r, c));
        }
    }
    return res;
}

Eigen::Matrix4d DHMatrix(std::array<double, 4> dh) {
    Eigen::Matrix4d m;
    m << cos(dh[3]), -sin(dh[3]) * cos(dh[1]), sin(dh[3]) * sin(dh[1]), dh[0] * cos(dh[3]),
            sin(dh[3]), cos(dh[3]) * cos(dh[1]), -cos(dh[3]) * sin(dh[1]), dh[0] * sin(dh[3]),
            0, sin(dh[1]), cos(dh[1]), dh[2],
            0, 0, 0, 1.0;
    return m;
}

