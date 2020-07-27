#include <iostream>
#include <algorithm>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "point_xyz.hpp"
#include "io_xyz.hpp"

using namespace std;

// void generate_uniform_data(int num, std::vector<Point3d>& points) {
//     points.clear();
//     srand(time(nullptr)); 
//     for (int i = 0; i < num; i ++) {
//         double x = rand() * 1.0 / RAND_MAX;
//         double y = rand() * 1.0 / RAND_MAX;
//         points.push_back({x, y, .0});
//     }
// }

void generate_non_uniform_data(int num, std::vector<Point3d>& points) {
    points.clear();
    srand(time(nullptr)); 
    for (int i = 0; i < num * 0.7; i ++) {
        double x = rand() * 1.0 / RAND_MAX * 0.5;
        double y = rand() * 1.0 / RAND_MAX;
        points.push_back({x, y, .0});
    }
    for (int i = 0; i < num * 0.3; i ++) {
        double x = rand() * 1.0 / RAND_MAX * 0.5 + 0.5;
        double y = rand() * 1.0 / RAND_MAX;
        points.push_back({x, y, .0});
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " xxx(points num) xxx.npts(output)" << std::endl;
        return -1;
    }
    std::vector<Point3d> points;
    int num = atoi(argv[1]);
    //generate_uniform_data(num, points);
    generate_non_uniform_data(num, points);
    zjl::IO::write_point_cloud_to_npts_without_normals(argv[2], points);
    return 0;
}