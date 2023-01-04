#ifndef FKSOLVERINTERFACE_H
#define FKSOLVERINTERFACE_H

#include <queue>

#include <Eigen/Dense>
#include <Eigen/Core>

#include "urdf/model.h"
#include "RBLink.h"
#include "Kine.h"


class Kine;

class FKSolverInterface {
public:
    std::shared_ptr<Kine> chain;

    explicit FKSolverInterface(Kine &chain) {
        this->chain = std::make_shared<Kine>(chain);
    }

    virtual ~FKSolverInterface() = default;

    virtual void solve(std::vector<double> joints);
    virtual void solveLinksRange(std::string startLinkGenericName, std::string endLinkGenericName, std::vector<double> joints);

    void updateLinksWithChildren(std::string startLinkGenericName);
};

#endif //FKSOLVERINTERFACE_H
