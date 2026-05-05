#ifndef __reader__HPP
#define __reader__HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <lemon/list_graph.h>
#include <lemon/bfs.h>

using namespace std;
using namespace lemon;

// --------------------------------------------------------//
// This file hosts the list of instance related functions. // 
// --------------------------------------------------------//

// --------------- //
// Instance reader //
// --------------- //

// Function that takes an empty graph g, reads an instance from the literature and builds it in g
vector<ListGraph::Node> readInstance(ListGraph & g, const string & filename);

// OR

// ------------------ //
// Path graph creator //
// ------------------ //

// Function that takes an empty graph g and builds a path graph in it
void createPathGraph(ListGraph & g, int n, vector<ListGraph::Node> & nodes);

#endif