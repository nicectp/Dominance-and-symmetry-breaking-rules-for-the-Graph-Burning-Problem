#ifndef __generic_callback__hpp
#define __generic_callback__hpp

// CPLEX and Lemon headers
#include <ilcplex/ilocplex.h>
#include <lemon/list_graph.h>

// Own headers
#include "../tools/others.hpp"

// C++ libraries
#include <thread>
#include <mutex>
#include <iostream>
#include <vector>
#include <map>
#include <set>

// Typedefs
typedef std::vector<IloNum> IloNumVector;
typedef std::vector<IloNumVector> IloNumMatrix;
typedef IloCplex::Callback::Context Context;

static constexpr double EPS = 1e-6;

ILOSTLBEGIN

class MyGenericCallback : public IloCplex::Callback::Function {

    // ----------------- //
    // General variables //
    // ----------------- //
    const IloEnv & env; 

    // ----------------------- //
    // Model related variables //
    // ----------------------- //
    const IloArray<IloNumVarArray> & x;
    
    const std::vector<std::tuple<int,int,int,int>> & impls;

    // ---------------------------- //
    // Manage execution and control //
    // ---------------------------- //
    std::mutex mtx; // A mutex for synchronizing multi-thread operations
    int nbUserCuts; // Number of user cuts added
    int nbCutoffs;  // Number of cutoffs performed

    // ------ //
    // Others //
    // ------ //
    bool doCutoff;   
    bool doUserCuts;

    public:

    // ------------ //
    // Constructors //
    // ------------ //

    MyGenericCallback(const IloEnv & env_, const IloArray<IloNumVarArray> & x_, const std::vector<std::tuple<int,int,int,int>> & impls_, bool doCutoff_, bool doUserCuts_);

    // --------------- //
    // Main Operations //
    //---------------- //

    // CPLEX will call this method during the solution process at the places that we asked for
    void invoke(const IloCplex::Callback::Context & context) ILO_OVERRIDE;  

    // Solves the separation problems for a given fractional solution. @note Should only be called within relaxation context.
    void addUserCuts(const IloCplex::Callback::Context& context);

    // Retrieveus the maximum radius of the incumbent solution
    int getIncumbentMaxRadius(const IloCplex::Callback::Context& context);

    // Cuts off the node exploration. @note Should only be called within relaxation context.
    void cutOffNode(const IloCplex::Callback::Context& context);
    
    // ------------------------ //
    // Thread protected methods // 
    // ------------------------ //

    void incrementUserCuts(); // to keep track of all user cuts that we attempt to add (including the ones CPLEX decides not to add when using UseCutFilter)
    int getNbUserCuts();      // returns the number of user cuts that we attempted to add

    void incrementCutoffs(); // to keep track of all cutoffs that we attempt to perform
    int getNbCutoffs();      // returns the number of cutoffs that we attempted to perform

};


#endif


