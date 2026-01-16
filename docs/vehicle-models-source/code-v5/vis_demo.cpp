#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "planning/vis/vis_graph.hpp"
#include "planning/vis/geometry.hpp"

// Minimal JSON reader for maps: expects format with boundary + obstacles (rects/polys)
// For brevity, we parse a simple ad-hoc subset.

struct Map {
    geom::Pt start, goal; std::vector<geom::Poly> obstacles;
};

static bool load_map(const std::string& path, Map& M){
    std::ifstream in(path); if (!in) return false; std::string s((std::istreambuf_iterator<char>(in)), {});
    auto find = [&](const std::string& key){ return s.find(key); };
    auto get_number_after = [&](size_t pos){ pos = s.find(':', pos); size_t e=s.find_first_of(",}\n ", pos+1); return std::stod(s.substr(pos+1, e-pos-1)); };

    size_t ps = find("\"start\"");
    size_t px = find("x", ps); M.start.x = get_number_after(px);
    size_t py = find("y", px); M.start.y = get_number_after(py);

    size_t gs = find("\"goal\"");
    px = find("x", gs); M.goal.x = get_number_after(px);
    py = find("y", px); M.goal.y = get_number_after(py);

    // obstacles as list of polys: "polys": [ [ [x,y],... ], ... ]
    size_t po = find("\"polys\""); if (po==std::string::npos) return true;
    size_t arr = s.find('[', po);
    while (true){
        size_t poly_start = s.find('[', arr+1); if (poly_start==std::string::npos) break;
        size_t poly_end   = s.find(']', poly_start+1); if (poly_end==std::string::npos) break;
        if (s[poly_start+1] == ']') { arr = poly_end; continue; }
        geom::Poly P;
        size_t p = poly_start+1;
        while (true){
            size_t lb = s.find('[', p); if (lb>poly_end || lb==std::string::npos) break;
            size_t comma = s.find(',', lb+1); size_t rb = s.find(']', comma+1);
            double x = std::stod(s.substr(lb+1, comma-lb-1));
            double y = std::stod(s.substr(comma+1, rb-comma-1));
            P.push_back({x,y});
            p = rb+1;
        }
        M.obstacles.push_back(P);
        arr = poly_end;
        if (s[arr+1] != ',') break;
    }
    return true;
}

int main(int argc, char** argv){
    if (argc<3){ std::cerr << "usage: vis_demo <map.json> <out.graph.json>\n"; return 1; }
    Map M; if (!load_map(argv[1], M)){ std::cerr << "failed to load map\n"; return 1; }
    vis::ScenePolys scene; scene.obstacles = M.obstacles;
    auto G = vis::build_visibility_graph(M.start, M.goal, scene, 0.5);

    std::ofstream out(argv[2]);
    out << "{\n  \"nodes\": [\n";
    for (size_t i=0;i<G.nodes.size();++i){
        out << "    {\"x\": "<<G.nodes[i].x<<", \"y\": "<<G.nodes[i].y<<"}";
        out << (i+1<G.nodes.size()? ",\n" : "\n");
    }
    out << "  ],\n  \"edges\": [\n";
    for (size_t i=0;i<G.edges.size();++i){
        out << "    {\"u\": "<<G.edges[i].first<<", \"v\": "<<G.edges[i].second<<", \"w\": "<<G.w[i]<<"}";
        out << (i+1<G.edges.size()? ",\n" : "\n");
    }
    out << "  ]\n}\n";
    return 0;
}
