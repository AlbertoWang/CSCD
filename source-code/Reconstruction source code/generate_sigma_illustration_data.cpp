#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include "point_xyz.hpp"
#include "io_xyz.hpp"

using namespace std;

void generate_data(int point_num, std::vector<Point3d>& points) {
    points.clear();
    srand(time(0));
    int valid_point_number = 0;
    int x_segment_number = 100;
    double x_length = 2.0;
    double x_segment_width = x_length / x_segment_number;
    int point_num_per_segment = (int)std::ceil(point_num * 1.0 / x_segment_number);
    const double E = 2.71828;
    for (int x_i = 0; x_i < x_segment_number; x_i ++) {
        double x_aver = x_i * 1.0 / x_segment_number * x_length;
        double x_left = x_aver - x_segment_width * 0.5;
        double y_aver = 0.0;
        double y_bott = - 0.5 * pow(E, - x_aver * x_aver / (x_length * x_length * 0.35 * 0.35));
        double y_upper = 0.5 * pow(E, - x_aver * x_aver / (x_length * x_length * 0.35 * 0.35));

        for (int p_i = 0; p_i < point_num_per_segment; p_i ++) {
            double x_normalized = rand() * 1.0 / RAND_MAX;
            double y_normalized = rand() * 1.0 / RAND_MAX;
            double x = x_left + x_segment_width * x_normalized;

            double y = y_bott + (y_upper - y_bott) * y_normalized;
            double z = 0.0;
            points.push_back({x, y, z});
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " number(point num) xxx.npts(output file path)" << std::endl;
        return -1;
    }
    int points_num = atoi(argv[1]);
    auto output_path = argv[2];
    std::vector<Point3d> points;
    generate_data(points_num, points);
    zjl::IO::write_point_cloud_to_npts_without_normals(output_path, points);
    return 0;
}