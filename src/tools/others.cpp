#include "others.hpp"

std::map<std::pair<int,int>, std::set<int>> getAllBalls(int UB, lemon::ListGraph& g) {
    
    std::map<std::pair<int,int>, std::set<int>> balls;     // balls = [key, value] = [pair<int, int>, set<int>]
                                                           //    key = pair (v, r) of B_r[v]
    for (ListGraph::NodeIt v(g); v != INVALID; ++v) {      //    value = nodes covered by B_r[v]
        
        Bfs<ListGraph> bfs(g);
        bfs.run(v); 

        for (ListGraph::NodeIt u(g); u != INVALID; ++u) {         
            
            int dist = bfs.dist(u);                    
            if (dist > UB) continue;
            
            for (int r = dist; r <= UB; ++r) {        
                balls[{g.id(v), r}].insert(g.id(u));
            }
        }
    }
    return balls;
}


std::set<std::pair<int,int>> FirstDominaceRule(const std::map<std::pair<int,int>, std::set<int>> & balls, const std::map<int,int>& lex_ord) {
    
    std::set<std::pair<int,int>> unnecessaryBalls;

    for (const auto & [key_v, value_v] : balls) {
        int v   = key_v.first;
        int r_v = key_v.second;

        for (const auto & [key_u, value_u] : balls) {
            int u   = key_u.first;
            int r_u = key_u.second;

            if (r_u != r_v || u == v) continue;

            // Diff(u,v,r) = Br(u) \ Br(v)
            std::set<int> diff_uvr;
            std::set_difference(value_u.begin(), value_u.end(), value_v.begin(), value_v.end(), std::inserter(diff_uvr, diff_uvr.begin()));

            // if Diff(u,v,r) is empty -> B_r[u] is unnecessary
            if (diff_uvr.empty()) {
                
                if (value_u == value_v) {
                    if (lex_ord.at(u) < lex_ord.at(v)) {
                        unnecessaryBalls.insert({u, r_u});
                    }
                } 
                else {
                    unnecessaryBalls.insert({u, r_u});
                }
            }
        }
    }
    return unnecessaryBalls;
}


std::set<std::pair<int,int>> FirstDominaceRuleConsideringCovered(const std::map<std::pair<int,int>, std::set<int>> & balls, const std::map<int,int> & lex_ord, const std::set<int> & coveredNodes) {
    
    std::set<std::pair<int,int>> unnecessaryBalls;

    for (const auto & [key_v, value_v] : balls) {
        int v   = key_v.first;
        int r_v = key_v.second;

        for (const auto & [key_u, value_u] : balls) {
            int u   = key_u.first;
            int r_u = key_u.second;

            if (r_u != r_v || u == v) continue;

            // Diff = (B(u,r) \ B(v,r))
            std::set<int> diff_uvr;
            std::set_difference(value_u.begin(), value_u.end(), value_v.begin(), value_v.end(), std::inserter(diff_uvr, diff_uvr.begin()));

            // Diff = Diff \ C   (removing alredy covered nodes)
            for (auto it = diff_uvr.begin(); it != diff_uvr.end(); ) {
                if (coveredNodes.count(*it)) it = diff_uvr.erase(it);
                else ++it;
            }

            if (diff_uvr.empty()) {
                
                if (value_u == value_v) {
                    if (lex_ord.at(u) < lex_ord.at(v)) {
                        unnecessaryBalls.insert({u, r_u});
                    }
                } 
                else {
                    unnecessaryBalls.insert({u, r_u});
                }
            }
        }
    }
    return unnecessaryBalls;
}

// returns list of: (v, r, u, r') such that x(xi,ri)=1 => x(xj,rj)=0
std::vector<std::tuple<int,int,int,int>> SecondDominanceRule(const ListGraph & g, const std::map<std::pair<int,int>, std::set<int>>& balls){
    
    std::vector<std::tuple<int,int,int,int>> implications;

    for (const auto & [key_xi, value_xi] : balls) {
        
        int xi_id   = key_xi.first;
        int ri = key_xi.second;

        ListGraph::Node v = g.nodeFromId(xi_id);

        Bfs<ListGraph> bfs(g);
        bfs.run(v);

        for (const auto & [key_xj, value_xj] : balls) {
        
        int xj_id   = key_xj.first;
        int rj = key_xj.second;

            ListGraph::Node u = g.nodeFromId(xj_id);

            int d = bfs.dist(u);

            if (ri <= rj) continue;

            if (d < ri - rj) {
                implications.emplace_back(xi_id, ri, xj_id, rj);
            }
        }
    }
    return implications;
}


/*

std::set<std::pair<int,int>> getUnnecessaryBalls2(const std::map<std::pair<int,int>, std::set<int>> & balls) {
    
    std::set<std::pair<int,int>> unnecessaryBalls;

    for (const auto & [key_xi, value_xi] : balls) {
        int xi   = key_xi.first;
        int ri = key_xi.second;

        for (const auto & [key_xj, value_xj] : balls) {
            int xj   = key_xj.first;
            int rj = key_xj.second;

            if (ri <= rj) continue;
            if (xi == xj && ri == rj) continue;    

            // Diff = B_rj(xj) \ B_ri(xi)
            std::set<int> diff;
            std::set_difference(value_xj.begin(), value_xj.end(), value_xi.begin(), value_xi.end(), std::inserter(diff, diff.begin()));

            if (diff.empty()) {
                unnecessaryBalls.insert({xj, rj}); 
            }
        }
    }

    return unnecessaryBalls;
}



std::set<std::pair<int,int>> getUnnecessaryBalls22(ListGraph & g, const std::map<std::pair<int,int>, std::set<int>>& balls){
    
    std::set<std::pair<int,int>> unnecessary;

    for (const auto & [key_xi, value_xi] : balls) {
        
        int xi_id   = key_xi.first;
        int ri = key_xi.second;

        ListGraph::Node xi = g.nodeFromId(xi_id);

        Bfs<ListGraph> bfs(g);
        bfs.run(xi);

        for (const auto & [key_xj, value_xj] : balls) {
            
            const int xj_id = key_xj.first;
            const int rj    = key_xj.second;

            ListGraph::Node xj = g.nodeFromId(xj_id);

            int dist = bfs.dist(xj);

            if (ri <= rj) continue;

            if (dist < ri - rj) {
                unnecessary.insert(key_xj);
            }
        }
    }

    return unnecessary;
}


*/