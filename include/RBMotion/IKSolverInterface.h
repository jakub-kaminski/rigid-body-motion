#ifndef MYLIBRARYPROJECT_IKSOLVERINTERFACE_H
#define MYLIBRARYPROJECT_IKSOLVERINTERFACE_H


#include "Kine.h"

class Kine;

class IKSolverInterface {
public:
    std::shared_ptr<Kine> chain;

    explicit IKSolverInterface(Kine &chain) {
        this->chain = std::make_shared<Kine>(chain);
    }

    virtual ~IKSolverInterface() = default;

    virtual std::vector<double> solve(Eigen::Matrix4d) {return {};}
    virtual std::vector<double> solve(Eigen::Matrix4d, Eigen::Matrix4d) {return {};}
};


#endif //MYLIBRARYPROJECT_IKSOLVERINTERFACE_H
