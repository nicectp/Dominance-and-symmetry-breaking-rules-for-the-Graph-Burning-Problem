#include "legacyCallback.hpp"
#include "../tools/others.hpp"

MyBranchCallbackI::MyBranchCallbackI(IloEnv env, const IloArray<IloNumVarArray>& x_, const std::map<std::pair<int,int>, std::set<int>>& allBalls_, const std::map<int,int>& lex_ord_) : IloCplex::BranchCallbackI(env), x(x_), allBalls(allBalls_), lex_ord(lex_ord_) {}


IloCplex::CallbackI* MyBranchCallbackI::duplicateCallback() const {
    return (new (getEnv()) MyBranchCallbackI(getEnv(), x, allBalls, lex_ord));
}


void MyBranchCallbackI::main() {
    if (getNbranches() == 0) return;
    addFixings();
}


std::set<int> MyBranchCallbackI::getCoveredNodes() const {
    
    std::set<int> coveredNodes;

    for (const auto& [key_v, value_v] : allBalls) {
        int u = key_v.first;
        int r = key_v.second;

        IloNum lb = getLB(x[u][r]);
        IloNum ub = getUB(x[u][r]);

        if (lb >= 1.0 - TOL && ub <= 1.0 + TOL) {
            coveredNodes.insert(value_v.begin(), value_v.end());
        }
    }
    return coveredNodes;
}


void MyBranchCallbackI::addFixings() {
    
    std::set<int> coveredNodes = getCoveredNodes();

    std::map<std::pair<int,int>, std::set<int>> activeBalls;

    for (const auto & [key, nodes] : allBalls) {
        int u = key.first;
        int r = key.second;
        if (getUB(x[u][r]) > TOL) {
            activeBalls.emplace(key, nodes);
        }
    }

    if (activeBalls.empty()) return;

    std::set<std::pair<int,int>> unnecessaryBalls1DR = FirstDominaceRuleConsideringCovered(activeBalls, lex_ord, coveredNodes);

    if (unnecessaryBalls1DR.empty()) return;

    const IloInt nb = getNbranches();

    for (IloInt b = 0; b < nb; ++b) {
        
        IloNumVarArray vars(getEnv());
        IloNumArray bounds(getEnv());
        IloCplex::BranchDirectionArray dirs(getEnv());

        IloNum estimate = getBranch(vars, bounds, dirs, b);

        for (const auto & [u, r] : unnecessaryBalls1DR) {

            if (getUB(x[u][r]) <= TOL) continue; // if the variable is already fixed to 0, skip it
            
            vars.add(x[u][r]);
            bounds.add(0.0);
            dirs.add(IloCplex::BranchDown); 
        }

        makeBranch(vars, bounds, dirs, estimate);
    }
}