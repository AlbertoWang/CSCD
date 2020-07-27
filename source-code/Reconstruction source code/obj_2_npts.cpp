#include <iostream>
#include <vector>
#include <algorithm>

#include "point_xyz.hpp"
#include "io_xyz.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " xxx.obj(from) xxx.npts(to) "
        << std::endl;
        return -1;
    }
    std::vector<Point3d> points;
    zjl::IO::read_point_cloud_from_obj_without_normals(argv[1], points);
    zjl::IO::write_point_cloud_to_npts_without_normals(argv[2], points);
    return 0;
}