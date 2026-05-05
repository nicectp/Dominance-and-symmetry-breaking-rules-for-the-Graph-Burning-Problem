#ifndef __legacy_callback__hpp
#define __legacy_callback__hpp

#include <ilcplex/ilocplex.h>
#include <map>
#include <set>
#include <utility>

ILOSTLBEGIN

static constexpr double TOL = 1e-5;

class MyBranchCallbackI : public IloCplex::BranchCallbackI {

    const IloArray<IloNumVarArray>& x;
    const std::map<std::pair<int,int>, std::set<int>> & allBalls;
    const std::map<int,int> & lex_ord;

    public:

    MyBranchCallbackI(IloEnv env, const IloArray<IloNumVarArray>& x_, const std::map<std::pair<int,int>, std::set<int>>& allBalls_, const std::map<int,int>& lex_ord_);

    IloCplex::CallbackI * duplicateCallback() const override;

    void main() override;

    std::set<int> getCoveredNodes() const;
    void addFixings();
};

#endif



/*
.hpp:

// Function that identifies nodes that have alredy been burned (covered) due to previously made decisions in the B&B tree
    //std::set<int> getCoveredNodes(const IloCplex::Callback::Context& context) const;

    // Adding the preprocessing fixings to the INTERNAL nodes of the B&B tree
    //void addFixings(const IloCplex::Callback::Context& context) const;

.cpp:

std::set<int> MyGenericCallback::getCoveredNodes(const IloCplex::Callback::Context & context) const {

    std::set<int> coveredNodes; 
    
    for (IloInt i = 0; i < x.getSize(); ++i) {
        
        for (IloInt j = 0; j < x[i].getSize(); ++j) {

            IloNum lb = context.getLocalLB(x[i][j]);
            IloNum ub = context.getLocalUB(x[i][j]);

            if (lb == 1.0 && ub == 1.0) {

                for (const auto & [key_v, value_v] : allBalls){

                    if(key_v.first == i && key_v.second == j){
                        coveredNodes.insert(value_v.begin(), value_v.end());
                    }
                }
            }
            
        }
    }
    return coveredNodes;
}

void Callback::addFixings(const IloCplex::Callback::Context& context) const {

    std::set<int> coveredNodes = getCoveredNodes(context);

    std::set<std::pair<int,int>> unnecessaryBalls1DR = getUnnecessaryBalls1ConsideringCovered(allBalls, lex_ord, coveredNodes);

    int nbBranches = getNbranches(); 

    for (int b = 0; b < nbBranches; ++b) {

        IloNumVarArray vars(env);
        IloNumArray bounds(env);
        IloCplex::BranchDirectionArray dirs(env);

        getBranch(vars, bounds, dirs, b);  

        for (const auto& [u, r] : unnecessaryBalls1DR) {
            vars.add(x[u][r]);
            bounds.add(0.0);
            dirs.add(IloCplex::BranchDown);
        }

        makeBranch(vars, bounds, dirs, getObjValue());

    }
}

void Callback::addFixings(const IloCplex::Callback::Context& context) const {

    std::set<int> coveredNodes = getCoveredNodes(context);

    std::set<std::pair<int,int>> unnecessaryBalls1DR = getUnnecessaryBalls1ConsideringCovered(allBalls, lex_ord, coveredNodes);

    if (unnecessaryBalls1DR.empty()) return;

    IloNumVarArray vars(env);
    IloNumArray bounds(env);
    IloCplex::BranchDirectionArray dirs(env);

    for (const auto& [u, r] : unnecessaryBalls) {
        vars.add(x[u][r]);
        bounds.add(0.0);
        dirs.add(IloCplex::BranchDown);
    }

    context.makeBranch(vars, bounds, dirs, context.getRelaxationObjective());

}


*/