#include <iostream>
#include "point_xyz.hpp"
#include "io_xyz.hpp"
#include "normalize_xyz.hpp"

using namespace std;

void normalize(std::vector<Point3d>& mesh_points) {
    if (mesh_points.empty()) {
        cerr << "Error: no points" << endl;
        exit(-1);
    }
    const double LD = std::numeric_limits<double>::lowest();
    const double MD = std::numeric_limits<double>::max();
    Point3d max_point(LD, LD, LD);
    Point3d min_point(MD, MD, MD);
    for (const auto& kv : mesh_points) {
        max_point.x = std::max(kv.x, max_point.x);
        max_point.y = std::max(kv.y, max_point.y);
        max_point.z = std::max(kv.z, max_point.z);
            // ==
        min_point.x = std::min(kv.x, min_point.x);
        min_point.y = std::min(kv.y, min_point.y);
        min_point.z = std::min(kv.z, min_point.z);
    }
    
    auto center = (max_point + min_point) * 0.5;
    auto diagonal = (max_point - min_point) * 0.5;

    //double scaling = 1.0 / std::max({diagonal.x, diagonal.y, diagonal.z});
    double scaling = 1.0 / 37.5;
    for (auto& point : mesh_points) {
        // point = point - center;
        point = point * scaling;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " xx.npts(in) xxx.npts(out)" << endl;
        return -1;
    }
    vector<Point3d> points;
    zjl::IO::read_point_cloud_from_npts_without_normals(argv[1], points);
    normalize(points);
    zjl::IO::write_point_cloud_to_npts_without_normals(argv[2], points);
    return 0;
}