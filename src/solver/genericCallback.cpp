#include "genericCallback.hpp"

MyGenericCallback::MyGenericCallback(
    const IloEnv & env_,
    const IloArray<IloNumVarArray> & x_,
    const std::vector<std::tuple<int,int,int,int>> & impls_,
    bool doCutoff_,
    bool doUserCuts_
) : env(env_),
    x(x_),
    impls(impls_),
    nbUserCuts(0),
    nbCutoffs(0),
    doCutoff(doCutoff_),
    doUserCuts(doUserCuts_) {}

    
void MyGenericCallback::invoke(const Context & context) {

    switch (context.getId()) {

        case Context::Id::Relaxation:
            if (doUserCuts == true) {
                addUserCuts(context); // Add user cuts
            }
            if (doCutoff == true) {
            cutOffNode(context); // Cutoff node exploration      
            }
            break;

        default:
            break;
    }
}


void MyGenericCallback::addUserCuts(const IloCplex::Callback::Context& context){

    if (context.getId() != IloCplex::Callback::Context::Id::Relaxation){
        throw IloCplex::Exception(-1, "ERROR: Trying to get relaxation solution outside Relaxation context.");
    }

    for (const auto & tuple : impls) { 
        
        int xi, ri, xj, rj;
        std::tie(xi, ri, xj, rj) = tuple;

        double v = context.getRelaxationPoint(x[xi][ri]);
        double u = context.getRelaxationPoint(x[xj][rj]);

        if (v + u > 1.0 + EPS) { 

            IloRange cut(env, -IloInfinity, x[xi][ri] + x[xj][rj], 1.0);
            context.addUserCut(cut, IloCplex::UseCutFilter, IloFalse);  // OBS: when using UseCutFIlter some identified cuts may not be added, CPLEX will decide whether to add them or not 
            cut.end();                                                 // thus nbUserCuts will reflect the number of cuts that we attempted to add, not the exact number of cuts that were actually added

            incrementUserCuts();
        }
    }
}


int MyGenericCallback::getIncumbentMaxRadius(const IloCplex::Callback::Context& context) {

    int maxRadius = -1;

    for (int u = 0; u < x.getSize(); ++u) {
        
        for (int r = 0; r < x[u].getSize(); ++r) {
            
            double incumbent = context.getIncumbentValue(x[u][r]);
            
            if (incumbent > 1 - EPS) {
                maxRadius = std::max(maxRadius, r);
            }
        }
    }

    return maxRadius;
}


void MyGenericCallback::cutOffNode(const IloCplex::Callback::Context& context) {

    if (context.getId() != IloCplex::Callback::Context::Id::Relaxation){
        throw IloCplex::Exception(-1, "ERROR: Trying to access relaxation information outside Relaxation context.");
    }

    // Checking if there is an incumbent solution
    if (!context.getIntInfo(IloCplex::Callback::Context::Info::Feasible)) {
        return;
    }

    int rBest = getIncumbentMaxRadius(context);

    double currentRelaxation = context.getRelaxationObjective();
    double threshold = pow(2.0, rBest);  // 2^rBest

    if (currentRelaxation >= threshold - EPS) {
        
        std::cout << "Cutoff successfully performed!" << std::endl;

        context.pruneCurrentNode();
        incrementCutoffs();
    }
}


void MyGenericCallback::incrementUserCuts() {
    std::lock_guard<std::mutex> lock(mtx);
    ++nbUserCuts;
}


int MyGenericCallback::getNbUserCuts() {
    std::lock_guard<std::mutex> lock(mtx);
    return nbUserCuts;
}


void MyGenericCallback::incrementCutoffs() {
    std::lock_guard<std::mutex> lock(mtx);
    ++nbCutoffs;
}


int MyGenericCallback::getNbCutoffs() {
    std::lock_guard<std::mutex> lock(mtx);
    return nbCutoffs;
}