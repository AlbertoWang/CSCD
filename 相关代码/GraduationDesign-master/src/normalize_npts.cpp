#include <iostream>
#include "point_xyz.hpp"
#include "io_xyz.hpp"
#include "normalize_xyz.hpp"

using namespace std;

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " xx.npts(in) xxx.npts(out)" << endl;
        return -1;
    }
    vector<Point3d> points;
    zjl::IO::read_point_cloud_from_npts_without_normals(argv[1], points);
    zjl::Normalizer::normalize(points);
    zjl::IO::write_point_cloud_to_npts_without_normals(argv[2], points);
    return 0;
}