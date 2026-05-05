#include "model.hpp"

#include <iostream>
#include <cmath>

using namespace lemon;

// --------------------- //
//      Constructors     //
// --------------------- //

Model::Model(IloEnv & env_, const std::string & filename, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_ ) : callback(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(0), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
    
    std::cout << "=> Building model ... " << std::endl;

    // reading graph instance from file
    nodes = readInstance(g, filename);

    // compute parameters
    computeUB();
    computeBalls();
    computeLexOrder();

    // build formulation
    setVariables();
    setObjective();
    setConstraints();

    if (usePreprocessing){
        addPreprocessingFixings();
    }

    setCplexParameters();
    
    std::cout << "\t Model was correctly built ! \n" << std::endl;
}

Model::Model(IloEnv & env_, int n, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_) : callback(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(0), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
   
    std::cout << "=> Building model ... " << std::endl;

    // creating a path graph
    createPathGraph(g, n, nodes);

    // compute parameters
    computeUB();
    computeBalls();
    computeLexOrder();

    // build formulation
    setVariables();
    setObjective();
    setConstraints();

    if (usePreprocessing){
        addPreprocessingFixings();
    }

    setCplexParameters();

    std::cout << "\t Model was correctly built ! \n" << std::endl;       
}

Model::Model(IloEnv & env_, const std::string & filename, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_, int ubOverride) : callback(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(ubOverride), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
    
    std::cout << "=> Building model ... " << std::endl;

    // reading graph instance from file
    nodes = readInstance(g, filename);

    // compute parameters
    computeUB();
    computeBalls();
    computeLexOrder();

    // build formulation
    setVariables();
    setObjective();
    setConstraints();

    if (usePreprocessing){
        addPreprocessingFixings();
    }

    setCplexParameters();
    
    std::cout << "\t Model was correctly built ! \n" << std::endl;
}

Model::Model(IloEnv & env_, int n, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_, int ubOverride) : callback(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(ubOverride), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
   
    std::cout << "=> Building model ... " << std::endl;

    // creating a path graph
    createPathGraph(g, n, nodes);

    // compute parameters
    computeUB();
    computeBalls();
    computeLexOrder();

    // build formulation
    setVariables();
    setObjective();
    setConstraints();

    if (usePreprocessing){
        addPreprocessingFixings();
    }

    setCplexParameters();

    std::cout << "\t Model was correctly built ! \n" << std::endl;       
}

// --------------------------------- //
//      Computing the parameters     //
// --------------------------------- //

void Model::computeUB() {
    if (UB_override > 0) UB = UB_override;
    else UB = std::ceil(std::sqrt(countNodes(g)));
}

void Model::computeBalls() {
    allBalls = getAllBalls(UB, g);
}

void Model::computeLexOrder() {
    lex_ord.clear();
    for (size_t i = 0; i < nodes.size(); ++i) {
        lex_ord[g.id(nodes[i])] = static_cast<int>(i);
    }
}

// -------------------- //
//       Variables      //
// -------------------- //

void Model::setVariables() {

    // R
    R = IloNumVar(env, 0, UB, ILOINT, "R");
    model.add(R);

    // x[v][r]
    x = IloArray<IloNumVarArray>(env, countNodes(g));

    for (ListGraph::NodeIt v(g); v != INVALID; ++v) {
        
        x[g.id(v)] = IloNumVarArray(env, UB+1);

        for (int r = 0; r <= UB; ++r) {
            
            std::string name = "x_" + std::to_string(g.id(v)) + "_" + std::to_string(r);

            x[g.id(v)][r] = IloNumVar(env, 0, 1, ILOBOOL, name.c_str());
            
            model.add(x[g.id(v)][r]);
        }
    }
}

// -------------------- //
//       Objective      //
// -------------------- //

void Model::setObjective() {
    obj = IloObjective(env, R, IloObjective::Minimize);
    model.add(obj);
}

// ---------------------- //
//       Constraints      //
// ---------------------- //

void Model::setConstraints() {

    // ---------------- Constraint 1 ---------------- //
    for (int r = 0; r <= UB; ++r) {

        IloExpr expr(env);
        
        for (ListGraph::NodeIt v(g); v != INVALID; ++v) {
            expr += x[g.id(v)][r];
        }

        IloExpr lhs = R - r * expr;
        model.add(IloRange(env, 0, lhs, IloInfinity));

        lhs.end();
        expr.end();
    }

    // ---------------- Constraint 2 ---------------- //
    for (ListGraph::NodeIt v(g); v != INVALID; ++v) {

        IloExpr expr(env);

        for (int r = 0; r <= UB; ++r) {
            expr += x[g.id(v)][r];
        }

        model.add(IloRange(env, -IloInfinity, expr, 1));
        expr.end();
    }

    // ---------------- Constraint 3 ---------------- //
    for (int r = 0; r <= UB; ++r) {

        IloExpr expr(env);

        for (ListGraph::NodeIt v(g); v != INVALID; ++v) {
            expr += x[g.id(v)][r];
        }

        model.add(IloRange(env, -IloInfinity, expr, 1));
        expr.end();
    }

    // ---------------- Constraint 4 ---------------- //
    for (ListGraph::NodeIt v(g); v != INVALID; ++v) {

        IloExpr expr(env);

        for (int r = 0; r <= UB; ++r) {
            for (int u : allBalls[{g.id(v), r}]) {
                expr += x[u][r];
            }
        }

        model.add(IloRange(env, 1, expr, IloInfinity));
        expr.end();
    }
}

// --------------------------- //
//    Preprocessing Fixings    //
// --------------------------- //

// Adding the preprocessing fixings
void Model::addPreprocessingFixings() {

    std::set<std::pair<int,int>> unnecessaryBalls= FirstDominaceRule(allBalls, lex_ord);

    for (const auto & [u, r] : unnecessaryBalls) {
        model.add(x[u][r] == 0);
    }
}

// ------------------------ //
//     CPLEX Parameters     //
// ------------------------ //

void Model::setCplexParameters(){

    // Setting threads to 1
    cplex.setParam(IloCplex::Param::Threads, 1);
    
    // Desabling cut generation at root node
    //cplex.setParam(IloCplex::Param::MIP::Limits::CutPasses, -1); 

    // ------------------ //
    // Building callbacks //
    // ------------------ //

    // Branching Callback
    if(useBranchingCallback == true){
        cplex.use(new (env) MyBranchCallbackI(env, x, allBalls, lex_ord));
    }

    // Cutoff callback
    if (useCutOffCallback == true && useUserCutCallback == false) {
        throw std::invalid_argument("Invalid configuration: CutOff callback is not allowed in the original model.");
    }

    // User cut callback
    if(useUserCutCallback == true && useCutOffCallback == false){
        
        impls = SecondDominanceRule(g, allBalls); 
        
        callback = new MyGenericCallback(env, x, impls, useCutOffCallback, useUserCutCallback);

        // Defining contexts under which the callback will be executed
        auto context = IloCplex::Callback::Context::Id::Relaxation;

        // Using callback within the defined contexts
        cplex.use(callback, context);  

    }
}

// --------------- //
//      Solve      //
// --------------- //

void Model::run() {
    
    //cplex.exportModel("model.lp");

    double startTime = cplex.getCplexTime();
    double startTicks = cplex.getDetTime();
	
    cplex.solve();

    if(useUserCutCallback == true && callback != nullptr){
        std::cout << "\n\n=> Quantity of proposed User Cuts: " << callback->getNbUserCuts() << "\n";
    }

	// Get final execution time and ticks
	time = cplex.getCplexTime() - startTime;
    ticks = cplex.getDetTime() - startTicks;
}

void Model::printResult() const {

    std::cout << "\n=> Printing solution ..." << std::endl;

    std::cout << "\nObjective value = " << cplex.getObjValue() << "\n\n";

    for (ListGraph::NodeIt v(g); v != INVALID; ++v) {
        int vid = g.id(v);
        for (int r = 0; r <= UB; ++r) {
            if (cplex.getValue(x[vid][r]) > 0.5) {
                std::cout << "x[" << vid << "][" << r << "] = " << cplex.getValue(x[vid][r]) << "\n";
            }
        }
    }

    std::cout << "\n=> Total time spent by the solver: " << time << " s" << std::endl;
    std::cout << "=> Total deterministic time spent by the solver (ticks): " << ticks << std::endl;
    std::cout << "=> Nodes explored in the B&B tree: " << cplex.getNnodes() << std::endl << std::endl;
    std::cout << std::endl;
}

// ------- //
// Getters //
// ------- //

double Model::getObjValue() const {
    return cplex.getObjValue();
}

double Model::getTime() const {
    return time;
}

double Model::getTicks() const {
    return ticks;
}

int Model::getNodesExplored() const {
    return cplex.getNnodes();
}

int Model::getUserCuts() const {
    if (callback != nullptr) {
        return callback->getNbUserCuts();
    }
    return 0;
}

// ----------------- //
//     Destructor    //
// ----------------- //

Model::~Model(){
    if (callback != nullptr) {
        delete callback;
    }
}