#include "perturbedModel.hpp"

#include <iostream>
#include <cmath>

using namespace lemon;

// --------------------- //
//      Constructors     //
// --------------------- //

PerturbedModel::PerturbedModel(IloEnv & env_, const std::string & filename, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_) : callback_1(nullptr), callback_2(nullptr), callback_3(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(0), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
    
    std::cout << "=> Building model ... " << std::endl;

    // reading graph instance from file
    nodes = readInstance(g, filename);

    // compute parameters
    computeUB();
    computeCosts();
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
    
    std::cout << "\t Pertubated Model was correctly built ! \n" << std::endl;
}

PerturbedModel::PerturbedModel(IloEnv & env_, int n, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_) : callback_1(nullptr), callback_2(nullptr), callback_3(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(0), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
   
    std::cout << "=> Building model ... " << std::endl;

    // creating a path graph
    createPathGraph(g, n, nodes);

    // compute parameters
    computeUB();
    computeCosts();
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

    std::cout << "\t PerturbedModel was correctly built ! \n" << std::endl;       
}

PerturbedModel::PerturbedModel(IloEnv & env_, const std::string & filename, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_, int ubOverride) : callback_1(nullptr), callback_2(nullptr), callback_3(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(ubOverride), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
    
    std::cout << "=> Building model ... " << std::endl;

    // reading graph instance from file
    nodes = readInstance(g, filename);

    // compute parameters
    computeUB();
    computeCosts();
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
    
    std::cout << "\t PerturbedModel was correctly built ! \n" << std::endl;
}

PerturbedModel::PerturbedModel(IloEnv & env_, int n, bool usePreprocessing_, bool useBranchingCallback_, bool useCutOffCallback_, bool useUserCutCallback_, int ubOverride) : callback_1(nullptr), callback_2(nullptr), callback_3(nullptr), env(env_), model(env), cplex(model), obj(env), UB_override(ubOverride), usePreprocessing(usePreprocessing_), useBranchingCallback(useBranchingCallback_), useCutOffCallback(useCutOffCallback_), useUserCutCallback(useUserCutCallback_), R(env), x(env), time(0.0), ticks(0.0) {
   
    std::cout << "=> Building model ... " << std::endl;

    // creating a path graph
    createPathGraph(g, n, nodes);

    // compute parameters
    computeUB();
    computeCosts();
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

    std::cout << "\t PerturbedModel was correctly built ! \n" << std::endl;       
}

// --------------------------------- //
//      Computing the parameters     //
// --------------------------------- //

void PerturbedModel::computeUB() {
    if (UB_override > 0) UB = UB_override;
    else UB = std::ceil(std::sqrt(countNodes(g)));
}

void PerturbedModel::computeCosts() {
     for (int r = 0; r <= UB; r++) {
        c.push_back(1LL << r); // c[r] = 2^r
    }
}

void PerturbedModel::computeBalls() {
    allBalls = getAllBalls(UB, g);
}

void PerturbedModel::computeLexOrder() {
    lex_ord.clear();
    for (size_t i = 0; i < nodes.size(); ++i) {
        lex_ord[g.id(nodes[i])] = static_cast<int>(i);
    }
}

// -------------------- //
//       Variables      //
// -------------------- //

void PerturbedModel::setVariables() {

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

void PerturbedModel::setObjective() {

    IloExpr expr(env);

    for (ListGraph::NodeIt v(g); v != INVALID; ++v) {
        
        for (int r = 0; r <= UB; ++r) {
            expr += c[r] * x[g.id(v)][r];
        }
    }
    
    obj = IloObjective(env, expr, IloObjective::Minimize);
    model.add(obj);
    expr.end();
}

// ---------------------- //
//       Constraints      //
// ---------------------- //

void PerturbedModel::setConstraints() {

    // ---------------- Constraint 1 ---------------- //
    
    // R >= r * x[v][r] for all v, r
    // Erased from this model

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
void PerturbedModel::addPreprocessingFixings() {

    std::set<std::pair<int,int>> unnecessaryBalls= FirstDominaceRule(allBalls, lex_ord);

    for (const auto & [u, r] : unnecessaryBalls) {
        model.add(x[u][r] == 0);
    }

    /*impls = SecondDominanceRule(g, allBalls);

    for (const auto& [xi, ri, xj, rj] : impls) {
        model.add(x[xj][rj] + x[xi][ri] <= 1); // x(xi,ri)=1 => x(xj,rj)=0
    }*/
}

// ------------------------ //
//     CPLEX Parameters     //
// ------------------------ //

void PerturbedModel::setCplexParameters(){

    // Setting threads to 1
    cplex.setParam(IloCplex::Param::Threads, 1);
    
    // Desabling cut generation at root node
    //cplex.setParam(IloCplex::Param::MIP::Limits::CutPasses, -1); 

    // ------------------ //
    // Building callbacks //
    // ------------------ //

    // Branching callback
    if(useBranchingCallback == true){
        cplex.use(new (env) MyBranchCallbackI(env, x, allBalls, lex_ord));
    }

    // Cutoff callback
    if(useCutOffCallback == true && useUserCutCallback == false){ 
        
        std::vector<std::tuple<int,int,int,int>> empty;

        callback_1 = new MyGenericCallback(env, x, empty, useCutOffCallback, useUserCutCallback);

        // Defining contexts under which the callback will be executed
        auto context = IloCplex::Callback::Context::Id::Relaxation;

        // Using callback within the defined contexts
        cplex.use(callback_1, context);  

    }

    // User cut callback
    if(useUserCutCallback == true && useCutOffCallback == false){
        
        impls = SecondDominanceRule(g, allBalls); 
        
        callback_2 = new MyGenericCallback(env, x, impls, useCutOffCallback, useUserCutCallback);

        // Defining contexts under which the callback will be executed
        auto context = IloCplex::Callback::Context::Id::Relaxation;

        // Using callback within the defined contexts
        cplex.use(callback_2, context);  

    }

    // Both callbacks
    if (useUserCutCallback == true && useCutOffCallback == true) {
        
        impls = SecondDominanceRule(g, allBalls);
        
        callback_3 = new MyGenericCallback(env, x, impls, useCutOffCallback, useUserCutCallback);
        
        auto context = IloCplex::Callback::Context::Id::Relaxation;

        cplex.use(callback_3, context);
    }
}

// --------------- //
//      Solve      //
// --------------- //

void PerturbedModel::run() {
    
    //cplex.exportPerturbedModel("model.lp");

    double startTime = cplex.getCplexTime();
    double startTicks = cplex.getDetTime();
	
    cplex.solve();

    if (useUserCutCallback == true && callback_2 != nullptr) {
        std::cout << "\n\n=> Quantity of proposed User Cuts: " << callback_2->getNbUserCuts() << "\n";
    }

    if (useCutOffCallback == true && callback_1 != nullptr) {
        std::cout << "\n\n=> Quantity of cutoffs performed: " << callback_1->getNbCutoffs() << "\n";
    }

    if( useUserCutCallback == true && useCutOffCallback == true && callback_3 != nullptr) {
        std::cout << "\n\n=> Quantity of proposed User Cuts: " << callback_3->getNbUserCuts() << "\n";
        std::cout << "\n\n=> Quantity of cutoffs performed: " << callback_3->getNbCutoffs() << "\n";
    }

	// Get final execution time and ticks
	time = cplex.getCplexTime() - startTime;
    ticks = cplex.getDetTime() - startTicks;
}

void PerturbedModel::printResult() const {

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
    std::cout << "=> Total deterministic time spent by the solver (ticks): " << ticks << std::endl << std::endl;
    std::cout << "=> Nodes explored in the B&B tree: " << cplex.getNnodes() << std::endl << std::endl;
    std::cout << std::endl;
}

// ------- //
// Getters //
// ------- //

double PerturbedModel::getObjValue() const {
    return cplex.getObjValue();
}

double PerturbedModel::getTime() const {
    return time;
}

double PerturbedModel::getTicks() const {
    return ticks;
}

int PerturbedModel::getR() const {
    
    int rMax=0;
    
    for (int r = UB; r >= 0; --r) {
        
        for (ListGraph::NodeIt v(g); v != INVALID; ++v) {
            
            if (cplex.getValue(x[g.id(v)][r]) >= 1 - 1e-5) {
               
                if (r > rMax) rMax = r;
            }
        }
    }
    return rMax; 
}

int PerturbedModel::getUserCuts() const {
    if (callback_2 != nullptr) {
        return callback_2->getNbUserCuts();
    }

    else if (callback_3 != nullptr) {
        return callback_3->getNbUserCuts();
    }

    return 0;
}


int PerturbedModel::getCutOffs() const {
    if (callback_1 != nullptr) {
        return callback_1->getNbCutoffs();
    }
    
    else if (callback_3 != nullptr) {
        return callback_3->getNbCutoffs();
    }

    return 0;
}

int PerturbedModel::getNodesExplored() const {
    return cplex.getNnodes();
}

// ----------------- //
//     Destructor    //
// ----------------- //

PerturbedModel::~PerturbedModel(){
    if (callback_1 != nullptr) {
        delete callback_1;
    }

    if (callback_2 != nullptr) {
        delete callback_2;
    }

    if(callback_3 != nullptr) {
        delete callback_3;
    }
}