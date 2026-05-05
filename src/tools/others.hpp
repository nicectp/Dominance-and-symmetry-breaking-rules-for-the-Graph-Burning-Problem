#ifndef __others__HPP
#define __others__HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <lemon/list_graph.h>
#include <lemon/bfs.h>

#include <chrono>

using namespace lemon;

// ---------------------------------------------------------//
// This file hosts the list of all graph related functions. // 
// ---------------------------------------------------------//


// Function that calculates all the balls of the graph
std::map<std::pair<int,int>, std::set<int>> getAllBalls(int UB, lemon::ListGraph& g);


// Pre processing function to identify unnecessary balls of same radii //
//                       (1st Dominance Rule)                          //
std::set<std::pair<int,int>> FirstDominaceRule(const std::map<std::pair<int,int>, std::set<int>> & balls, const std::map<int,int>& lex_ord);

// 1st Dominance Rule considering alredy covered nodes during to previously made decisions in the B&B tree
std::set<std::pair<int,int>> FirstDominaceRuleConsideringCovered(const std::map<std::pair<int,int>, std::set<int>> & balls, const std::map<int,int> & lex_ord, const std::set<int> & coveredNodes);


// Pre processing function to identify unnecessary balls of different radii //
//                          (2st Dominance Rule)                            //
std::vector<std::tuple<int,int,int,int>> SecondDominanceRule(const ListGraph & g, const std::map<std::pair<int,int>, std::set<int>>& balls);

#endif