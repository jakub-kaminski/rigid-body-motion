#include "RBTreeParser.h"

#include <queue>

std::map<std::string, std::shared_ptr<RBLink>> parseToRBLinkMap(std::shared_ptr<urdf::UrdfModel> model){
    shared_ptr<urdf::Link> linkNow = model->root_link;

    std::map<std::string, std::shared_ptr<RBLink>> RBMap;
    std::queue<shared_ptr<urdf::Link>> q;
    q.push(linkNow);

    while (!q.empty()) {
        auto item = q.front();
        q.pop();
        auto rblink = std::make_shared<RBLink>(item);
        rblink->name = item->name;
        RBMap[item->name] = rblink;
        for (auto link: item->child_links) q.push(link);
    }

    for(const auto& item : RBMap){
        for(const auto& child_link : item.second->link->child_links){
            item.second->child_links.push_back(RBMap[child_link->name]);
        }
        if(item.second->link->parent_link != nullptr){
            item.second->parent_link = RBMap[item.second->link->parent_link->name];

            auto joint = item.second->link->parent_joint;
            item.second->q = Eigen::Quaterniond(
                    joint->parent_to_joint_transform.rotation.w,
                    joint->parent_to_joint_transform.rotation.x,
                    joint->parent_to_joint_transform.rotation.y,
                    joint->parent_to_joint_transform.rotation.z
            );

            item.second->t = Eigen::Vector3d{
                    joint->parent_to_joint_transform.position.x,
                    joint->parent_to_joint_transform.position.y,
                    joint->parent_to_joint_transform.position.z
            };


        }
        else{
            item.second->parent_link = nullptr;
        }
    }

    return RBMap;
}