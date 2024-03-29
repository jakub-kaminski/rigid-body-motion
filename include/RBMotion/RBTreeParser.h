//
// Created by jakub on 12/6/22.
//

#ifndef RBTREEPARSER_H
#define RBTREEPARSER_H

#include <Eigen/Dense>
#include <Eigen/Core>

#include <memory>
//#include <vector>

//#include <string>
#include "urdf/model.h"
#include "RBLink.h"

std::map<std::string, std::shared_ptr<RBLink>> parseToRBLinkMap(std::shared_ptr<urdf::UrdfModel>);

#endif //RBTREEPARSER_H
