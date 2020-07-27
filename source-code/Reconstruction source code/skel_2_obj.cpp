#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include "point_xyz.hpp"
#include "io_xyz.hpp"


int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " xxx.skel(from) xxx.obj(to) "
        << std::endl;
        return -1;
    }
    std::vector<Point3d> points;
    std::vector<Edge> edges;
    zjl::IO::read_graph_from_skel(argv[1], points, edges);
    zjl::IO::write_graph_to_obj(argv[2], points, edges);
    return 0;
}