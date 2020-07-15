#include <iostream>
#include <cmath>
#include <string>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "octree_xyz.hpp"
#include "wlop_xyz.hpp"

using namespace std;

#define CONST_E		2.7182818284590452354	/* e */

struct Point4d {
    double x;
    double y;
    double z;
    double r;

    Point4d (double m1, double m2, double m3, double m4):
        x(m1), y(m2), z(m3), r(m4) {}
};

struct Point3d {
    double x;
    double y;
    double z;

    Point3d(double px, double py, double pz):
    x(px), y(py), z(pz) {}

    double dot(const Point3d& ano) const{
        return this->x * ano.x + this->y * ano.y + this->z * ano.z;
    }

    double norm() const {
        return sqrt(x * x + y * y + z * z);
    }

    void normalize() {
        double len = this->norm();
        x /= len;
        y /= len;
        z /= len;
    }

    Point3d operator+ (const Point3d& pb) const {
        return {this->x + pb.x, this->y + pb.y, this->z + pb.z};
    }

    Point3d operator- (const Point3d& pb) const {
        return {this->x - pb.x, this->y - pb.y, this->z - pb.z};
    }

    Point3d operator* (const float r) const {
        return {this->x * r, this->y * r, this->z * r};
    }
};

using Vector3d = Point3d;
using Graph = std::unordered_map<int, std::unordered_set<int> >;

struct BoundingBox {
    Point3d center;
    Vector3d extent;

};


void write_points_to_npts(const vector<Point3d>& points, const string& file_name) {
    ofstream ofs(file_name);
    if (!ofs.is_open()) {
        cerr << "Error: failed to open file " << file_name << endl;
        return;
    }
    bool first_line = true;
    for (const auto& point : points) {
        if (first_line) first_line = false; else ofs << endl;
        ofs 
        << point.x << " " << point.y << " " << point.z ;
    }
}

void read_seeds_point_from_npts(const string& input_file, vector<Point4d>& seeds) {
    ifstream ifs(input_file);
    if (!ifs.is_open()) {
        cerr << "Error: can't open " << input_file << endl;
        return;
    }
    seeds.clear();
    string line;
    double x = 0, y = 0, z = 0, r = 0;
    while (std::getline(ifs, line)) {
        if (line.find("nan") != line.npos) continue;
        istringstream iss(line);
        if (!(iss >> x >> y >> z >> r)) {
            cerr << "Error: failed to read four float number from line: " << line << endl;
            break;
        }
        seeds.push_back({x, y, z, r});
    }

}

void generate_patches(const vector<Point3d>& raw_points, const vector<Point4d>& seed_points, vector< vector<Point3d>>& output_patches,  int points_num) {
    points_num = std::max(1, points_num);
    points_num = std::min((int)raw_points.size(), points_num);

    output_patches.clear();
    output_patches.reserve(seed_points.size() + 1);

    // for each seed, 
    using namespace zjl;
    OctreeXYZ<Point3d> octree(raw_points);
    cout << "Build spatial index for skeletal points ..." << endl;
    octree.build_index();
    cout << "OK" << endl;
    vector<int> neighboring_points;
    for (size_t i = 0; i < seed_points.size(); i ++) {
        const Point4d& seed_point = seed_points[i];
        const Point3d seed_position(seed_point.x, seed_point.y, seed_point.z);
        const double& seed_ball_radius = seed_point.r;
        octree.search_neighbors_within_distance(seed_position, seed_ball_radius, neighboring_points);
        // resample
        vector<Point3d> neighboring_underlying_surface;
        for (int point_index : neighboring_points) {
            neighboring_underlying_surface.push_back(raw_points[point_index]);
        }
        std::random_shuffle(neighboring_points.begin(), neighboring_points.end());
        vector<Point3d> samples;
        for (int i = 0; i < points_num; i ++) samples.push_back(raw_points[neighboring_points[i % neighboring_points.size()]]);
        cout << "Generating " << i << " patch" << endl;

        Wlop<Point3d> wlop(neighboring_underlying_surface);
        wlop.resample(samples, 10);

        // resample==
        // resample the samples to a set of #point_num points
        output_patches.push_back(samples);
    } 

}


void read_points_from_npts(const string& input_file, vector<Point3d> &points) {
    ifstream ifs(input_file);
    if (!ifs.is_open()) {
        cerr << "Error: can't open " << input_file << endl;
        return;
    }
    points.clear();
    string line;
    double x = 0, y = 0, z = 0;
    while (std::getline(ifs, line)) {
        if (line.find("nan") != line.npos) continue;
        istringstream iss(line);
        if (!(iss >> x >> y >> z)) {
            cerr << "Error: failed to read six float number from line: " << line << endl;
            break;
        }
        points.push_back({x, y, z});
    }
}

void write_patches_to_npts(const vector<vector<Point3d>>& patch_map, const string& output_file_path_stencil) {
    for (size_t i = 0; i < patch_map.size(); i ++) {
        ostringstream patch_file_name_oss;
        patch_file_name_oss << output_file_path_stencil << "_pc_" << i << ".npts";
        write_points_to_npts(patch_map[i], patch_file_name_oss.str());
    }
}

int main(int argc, char** argv) {
    if (argc < 5) {
        cerr << "Usage:" << argv[0] << " *.npts(input, point cloud file) *.npts(seed file)  ./dir/*(name  of patches, omitting suffix)  *(num of points in one patch)" << endl;
        return -1;
    }
    string points_file_path(argv[1]);
    string seeds_file_path(argv[2]);
    string patches_file_path(argv[3]);
    int points_num_per_patch = atoi(argv[4]);

    // prepare data container
    vector<Point3d> raw_point_cloud;

    // <seed, patch>
    // seed is Point4d (x, y, z, radius)
    vector<Point4d> seeds;
    vector<vector<Point3d>> output_patches;

    read_points_from_npts(points_file_path, raw_point_cloud);
    read_seeds_point_from_npts(seeds_file_path, seeds);

    generate_patches(raw_point_cloud, seeds, output_patches, points_num_per_patch);
    write_patches_to_npts(output_patches, patches_file_path);
    return 0;
}
