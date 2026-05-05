#include "reader.hpp"

vector<ListGraph::Node> readInstance(ListGraph & g, const string & filename) {

    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "The file could not be opened." << endl;
        exit(1);
    }

    int n, m;
    file >> n;
    file >> m;

    vector<ListGraph::Node> nodes(n);

    for (int i = 0; i < n; i++) {
        nodes[i] = g.addNode();
    }

    for (int k = 0; k < m; k++) {
        int u, v;
        file >> u >> v;
        g.addEdge(nodes[u - 1], nodes[v - 1]);
    }

    file.close();
    return nodes;
}


void createPathGraph(ListGraph & g, int n, vector<ListGraph::Node> & nodes) {
    
    nodes.resize(n);  
    
    for (int i = 0; i < n; i++) {                 
        nodes[i] = g.addNode();                  
    }

    for (int i = 0; i < n - 1; i++) {
        g.addEdge(nodes[i], nodes[i + 1]);
    }
}