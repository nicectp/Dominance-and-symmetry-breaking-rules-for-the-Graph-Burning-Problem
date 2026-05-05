#ifndef GB_MODEL_HPP
#define GB_MODEL_HPP

#include <map>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>

#include <ilcplex/ilocplex.h>
#include <lemon/list_graph.h>

#include "../tools/reader.hpp"
#include "../tools/others.hpp"

#include "genericCallback.hpp"
#include "legacyCallback.hpp"

ILOSTLBEGIN

class Model {

    private:
    
    MyGenericCallback * callback; // To handle User cut callback

    // --------------- //
    //      CPLEX      //
    // --------------- //
    const IloEnv & env;
    IloModel model;
    IloCplex cplex;

    // --------------------- //
    //       Formulation     //
    // --------------------- //
    IloObjective obj;

    // --------------- //
    //      LEMON      //
    // --------------- //
    lemon::ListGraph g;
    std::vector<lemon::ListGraph::Node> nodes;

    // --------------------- //
    //       Parameters      //
    // --------------------- //
    IloInt UB;
    IloInt UB_override; // to use the upper bound from heuristics

    std::map<std::pair<int,int>, std::set<int>> allBalls;
    std::map<int,int> lex_ord;

    std::vector<std::tuple<int,int,int,int>> impls;

    bool usePreprocessing;     // To add the preprocessing fixings to the model

    bool useBranchingCallback; // To use the legacy branching callback
    bool useCutOffCallback;    // To use the generic cutoff callback
    bool useUserCutCallback;   // To use the generic callback to add user cuts

    // -------------------- //
    //       Variables      //
    // -------------------- //
    IloNumVar R;
    IloArray<IloNumVarArray> x;

    // Manage execution and control //
	IloNum time;
    IloNum ticks;

    public:
    
    // --------------------- //
    //      Constructors     //
    // --------------------- //
    Model(IloEnv & env_, const std::string & filename, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_);  // constructor for literature instance, UB = ceil(sqrt(n))
    Model(IloEnv & env_, int n, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_);                         // constructor for path graph, UB = ceil(sqrt(n))

    Model(IloEnv & env_, const std::string & filename, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_, int ubOverride);  // constructor for literature instance,  UB = given upper bound
    Model(IloEnv & env_, int n, bool usePreprocessing_,  bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_, int ubOverride);                        // constructor for path graph, UB = given upper bound
    
    
    Model() = delete;

    // --------------------------------- //
    //      Computing the parameters     //
    // --------------------------------- //
    void computeUB();
    void computeBalls();
    void computeLexOrder();

    // --------------------- //
    //      Formulation      //
    // --------------------- //
    void setVariables();
    void setObjective();
    void setConstraints();
    void addPreprocessingFixings(); // Responsible for adding the preprocessing fixings

    void setCplexParameters(); // Setting CPLEX parameters

    // --------------- //
    //      Solve     //
    // --------------- //

    // Solves the MIP
    void run();

    // Displays the obtained solution
    void printResult() const;

    // ------- //
    // Getters //
    // ------- //

    // Retrieves the objective value
    double getObjValue() const;

    // Retrieves CPLEX execution time
    double getTime() const;

    // Retrieves the deterministic time (ticks)
    double getTicks() const;

    int getNodesExplored() const; // Retrieves the number of nodes explored in the B&B tree

    int getUserCuts() const;   // Retrieves the number of user cuts proposed during the search

    // ----------------- //
    //     Destructor    //
    // ----------------- //

    // free dynamic allocated memory
    ~Model();
};


#endif