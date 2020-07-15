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

using namespace std;

#define CONST_E		2.7182818284590452354	/* e */

struct Edge {
    int vi;
    int vj;

};

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

BoundingBox compute_bounding_box(const vector<Point3d>& raw_points) {
    if (raw_points.empty()) {
        cerr << "Error: no points" << endl;
        exit(-1);
    }
    Vector3d extent(0, 0, 0);
    Point3d center(0, 0, 0);
    const double LD = std::numeric_limits<double>::lowest();
    const double MD = std::numeric_limits<double>::max();
    Point3d max_point(LD, LD, LD);
    Point3d min_point(MD, MD, MD);
    for (const auto& kv : raw_points) {
        max_point.x = std::max(kv.x, max_point.x);
        max_point.y = std::max(kv.y, max_point.y);
        max_point.z = std::max(kv.z, max_point.z);
        // ==
        min_point.x = std::min(kv.x, min_point.x);
        min_point.y = std::min(kv.y, min_point.y);
        min_point.z = std::min(kv.z, min_point.z);
    }
    center = (max_point + min_point) * 0.5;
    extent = (max_point - min_point) * 0.5;
    return {center, extent};
}

void normalize_skeleton(const vector<Point3d>& mesh_points, vector<Point3d>& raw_skeleton_points, const double bbox_max_extent) {
    if (bbox_max_extent <= 0.0) {
        cout << "Error, bbox_max_extent: " << bbox_max_extent << endl;
        return;
    }
    auto bbox = compute_bounding_box(mesh_points);
    const double largest_extent = std::max({bbox.extent.x, bbox.extent.y, bbox.extent.z});
    const double ratio = bbox_max_extent * 1.0 / largest_extent;
    for (auto& skeleton_point : raw_skeleton_points) {
        skeleton_point = skeleton_point - bbox.center;
        skeleton_point = skeleton_point * ratio;
    }
}

void read_mesh_from_ply(const string& mesh_file_path, vector<Point3d>& mesh_points) {
    ifstream ifs(mesh_file_path);
    if (!ifs.is_open()) {
        cerr << "Error: can't open " << mesh_file_path << endl;
        return;
    }
    mesh_points.clear();
    string line;
    string keyword;
    double x = 0, y = 0, z = 0;
    int point_num = 0;
    while (std::getline(ifs, line)) {
        istringstream iss(line);
        if (!(iss >> keyword)) {
            cerr << "Error: failed to read the keyword from line: " << line << endl;
            continue;
        }
        if (keyword == "end_header" || keyword == "end_header\n") {
            cout << "Debug: Successfully get the end_header" << endl;
            break;
        } else if (keyword == "format") {
            iss >> keyword; 
            if (keyword != "ascii") {
                cout << "Warning: not an ascii file" << endl;
            }
        } else if (keyword == "element") {
            iss >> keyword; 
            if (keyword == "vertex") {
                iss >> point_num; 
                cout << "Debug: read the number of vertices: " << point_num << endl;
            }

        }
    }
    int read_cnt = 0;
    while (std::getline(ifs, line)) {
        if (line.find("nan") != line.npos) continue;
        istringstream iss(line);
        if (!(iss >> x >> y >> z)) {
            cerr << "Error: failed to read six float number from line: " << line << endl;
            break;
        }
        mesh_points.push_back({x, y, z});
        read_cnt += 1;
        if (read_cnt >= point_num) break;
    }
    cout << "Read " << mesh_points.size() << " points, " << endl;

}

void write_skeleton_to_obj(const string& output_skeleton_file_path, const vector<Point3d>& raw_skeleton_points, const vector<string>& lines_of_edges) {
    ofstream ofs(output_skeleton_file_path);
    if (!ofs.is_open()) {
        cerr << "Error: failed to open file " << output_skeleton_file_path << endl;
        return;
    }
    ofs << "# Written by zjl";
    for (const auto& point : raw_skeleton_points) {
        ofs 
        << endl << "v " << point.x << " " << point.y << " " << point.z ;
    }
    for (const auto& line : lines_of_edges) {
        ofs 
        << endl << line;
    }

}

void read_skeleton_from_obj(const string& input_file, vector<Point3d> &points, Graph& skeleton_graph, vector<string>& lines_of_edges) {
    ifstream ifs(input_file);
    if (!ifs.is_open()) {
        cerr << "Error: can't open " << input_file << endl;
        return;
    }
    points.clear();
    skeleton_graph.clear();
    lines_of_edges.clear();
    string line;
    string keyword;
    double x = 0, y = 0, z = 0;
    int vi = 0, vj = 0;
    int edge_cnt = 0;
    while (std::getline(ifs, line)) {
        if (line.find("nan") != line.npos) continue;
        istringstream iss(line);
        if (!(iss >> keyword)) {
            cerr << "Error: failed to read the keyword from line: " << line << endl;
            continue;
        }
        if (keyword == "#" || keyword == "o" || keyword == "mtllib") continue;
        if (keyword == "v") {
            if (!(iss >> x >> y >> z)) {
                cerr << "Error: failed to read six float number from line: " << line << endl;
                break;
            }
            points.push_back({x, y, z});
        }
        if (keyword == "l") {
            if (!(iss >> vi >> vj)) {
                cerr << "Error: failed to read two int number from line: " << line << endl;
                break;
            }
            skeleton_graph[vi - 1].insert(vj - 1);
            skeleton_graph[vj - 1].insert(vi - 1);
            lines_of_edges.push_back(line);
            edge_cnt += 1;
        }
    }
    cout << "Read " << points.size() << " points, " << edge_cnt << " edges." << endl;
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

int main(int argc, char** argv) {
    if (argc < 4) {
        cerr << "Usage:" << argv[0] << " *.ply(input mesh with raw size) *.obj(skel file that will be normalized)  *.obj(new skeleton file name)" << endl;
        return -1;
    }
    string mesh_file_path(argv[1]);
    string skeleton_file_path(argv[2]);
    string output_skeleton_file_path(argv[3]);

    // prepare data container
    vector<Point3d> mesh_points;

    vector<Point3d> raw_skeleton_points;
    Graph skeleton_graph;

    // <seed, patch>
    // seed is Point4d (x, y, z, radius)
    vector<Point4d> seeds;
    vector<vector<Point3d>> output_patches;
    vector<string> lines_of_edges;

    read_skeleton_from_obj(skeleton_file_path, raw_skeleton_points, skeleton_graph, lines_of_edges);
    read_mesh_from_ply(mesh_file_path, mesh_points);

    normalize_skeleton(mesh_points, raw_skeleton_points, 37.5);
    write_skeleton_to_obj(output_skeleton_file_path, raw_skeleton_points, lines_of_edges);
    return 0;
}
