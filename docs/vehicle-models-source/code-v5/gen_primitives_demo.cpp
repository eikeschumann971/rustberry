#include <iostream>
#include "planning/lattice/lattice_primitives.hpp"

int main(){
    lat::Params P; P.ds=0.05; P.L=2.7; P.k_max=0.25; P.sigma_max=0.3; P.lengths={0.4,0.8}; P.allow_reverse=true;
    auto prims = lat::generate_clothoid_rs_primitives(P);
    if (!lat::write_primitives_yaml("primitives/parking_clothoid_primitives.yaml", prims)){
        std::cerr<<"Failed to write primitives YAML\n"; return 1;
    }
    std::cout<<"Wrote primitives/parking_clothoid_primitives.yaml with "<<prims.size()<<" primitives\n";
    return 0;
}
