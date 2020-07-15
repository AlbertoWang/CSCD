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

    // bool operator==(const Edge& lhs, const Edge& rhs) const {
    //     return (lhs.vi == rhs.vi && lhs.vj == rhs.vj) ||
    //     (lhs.vi == rhs.vj && lhs.vj == rhs.vi);
    // }
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

void read_skeleton_from_obj(const string& input_file, vector<Point3d> &points, Graph& skeleton_graph) {
    ifstream ifs(input_file);
    if (!ifs.is_open()) {
        cerr << "Error: can't open " << input_file << endl;
        return;
    }
    points.clear();
    skeleton_graph.clear();
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
            edge_cnt += 1;
        }
    }
    cout << "Read " << points.size() << " points, " << edge_cnt << " edges." << endl;
}

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


void generate_skeletal_points(const vector<Point3d>& raw_points, const Graph& skeleton_graph, vector<Point3d>& output_points, float inter_ratio) {
    // clamp inter_ratio and noise_range_ratio to [0, 1]
    inter_ratio = std::max(0.f, inter_ratio);
    inter_ratio = std::min(1.f, inter_ratio);

    int inter_points_num = 10 * inter_ratio;
    if (inter_points_num <= 0) {
        cerr << "Warning: inter_points_num == 0" << endl;
        return;
    }
    output_points.clear();
    output_points.reserve((inter_points_num + 1) * raw_points.size());

    // declare equal and hash for customized struct
    auto edge_hash = [](const Edge& edge) {return 31 * edge.vi + edge.vj;};
    auto edge_equal = [](const Edge& lhs, const Edge& rhs) {
        return (lhs.vi == rhs.vi && lhs.vj == rhs.vj) || (lhs.vi == rhs.vj && lhs.vj == rhs.vi);
    };
    // use a hashset to skip duplicate edges when visiting edges in Graph
    unordered_set<Edge, decltype(edge_hash), decltype(edge_equal)> visited_edges(20, edge_hash, edge_equal);

    // compute the average edge length to determine the interpolation distance
    int edge_cnt = 0;
    double average_edge_length = 0.0;
    for (const auto& kv : skeleton_graph) {
        const Point3d& pi = raw_points[kv.first];
        for (const int adj_index : kv.second) {
            if (visited_edges.find({kv.first, adj_index}) != visited_edges.end()) continue;
            const Point3d& pj = raw_points[adj_index];
            average_edge_length += (pj - pi).norm();
            edge_cnt += 1;
            visited_edges.insert({kv.first, adj_index});
        }
    }
    if (edge_cnt == 0) {
        cout << "Error: edge_cnt  is 0" << endl;
        return ;
    }
    average_edge_length /= edge_cnt; 
    double interpolation_distance = average_edge_length / inter_points_num * 0.666;

    // start to sample points on the skeleton
    visited_edges.clear();
    for (const auto& kv : skeleton_graph) {
        const Point3d& pi = raw_points[kv.first];
        for (const int adj_index : kv.second) {
            if (visited_edges.find({kv.first, adj_index}) != visited_edges.end()) continue;
            const Point3d& pj = raw_points[adj_index];
            const Vector3d pipj = pj - pi;
            inter_points_num = pipj.norm() / interpolation_distance;
            for (int inter_i = 0; inter_i < inter_points_num; inter_i ++) {
                const Point3d& inter_candidate = pi + pipj * (1.0 * inter_i / inter_points_num);
                output_points.push_back(inter_candidate);
            }
            visited_edges.insert({kv.first, adj_index});
        }
    }
    cout << "Generate " << output_points.size() << " skeletal points" << endl;
}

void generate_patches(const vector<Point3d>& raw_skeleton_points, const Graph& skeleton_graph, const vector<Point3d>& skeletal_points, vector<pair<Point4d, vector<Point3d>>>& output_patches, int patch_num, int points_num) {
    // clamp patch_num and noise_range_ratio to [0, 300]
    patch_num = std::max(1, patch_num);
    patch_num = std::min(300, patch_num);
    points_num = std::max(1, points_num);
    points_num = std::min((int)skeletal_points.size(), points_num);

    output_patches.clear();
    output_patches.reserve(patch_num + 1);

    // randomly select #patch_num seed points
    // shuffle indices
    vector<int> indices(raw_skeleton_points.size());
    for (int i = 0; i < raw_skeleton_points.size(); i ++) indices[i] = i;
    std::random_shuffle(indices.begin(), indices.end());

    // for each seed,
    using namespace zjl;
    OctreeXYZ<Point3d> octree(skeletal_points);
    cout << "Build spatial index for skeletal points ..." << endl;
    octree.build_index();
    cout << "OK" << endl;
//return ;
    vector<int> neighboring_points;
    for (int i = 0; i < patch_num; i ++) {
        Point3d seed_position = raw_skeleton_points[indices[i]];
        octree.search_k_neighbors(points_num, seed_position, neighboring_points);
        double radius = (skeletal_points[neighboring_points.back()] - seed_position).norm();
        vector<Point3d> neighboring_points_positions;
        for (int point_index : neighboring_points) neighboring_points_positions.push_back(skeletal_points[point_index]);
        output_patches.emplace_back(Point4d(seed_position.x, seed_position.y, seed_position.z, radius), neighboring_points_positions);
        cout << "Generating " << i << " patch" << endl;
    }

}

void write_seeds_to_npts(const vector<pair<Point4d, vector<Point3d>>>& patch_map, const string& seed_path) {
    ofstream ofs(seed_path);
    if (!ofs.is_open()) {
        cerr << "Error: failed to open file " << seed_path << endl;
        return;
    }
    bool first_line = true;
    for (const auto& kv : patch_map) {
        if (first_line) first_line = false; else ofs << endl;
        ofs 
        << kv.first.x << " " << kv.first.y << " " << kv.first.z << " " << kv.first.r;
    }
}

void write_patches_to_npts(const vector<pair<Point4d, vector<Point3d>>>& patch_map, const string& output_file_path_stencil) {
    for (size_t i = 0; i < patch_map.size(); i ++) {
        ostringstream patch_file_name_oss;
        patch_file_name_oss << output_file_path_stencil << "_" << i << ".npts";
        write_points_to_npts(patch_map[i].second, patch_file_name_oss.str());
    }
}

int main(int argc, char** argv) {
    if (argc < 7) {
        cerr << "Usage:" << argv[0] << " *.obj(input, skeleton graph) *.npts(a list of seeds)  ./dir/*(name  of patches, omitting suffix)  *(num of patches) *(num of points in one patch) *(inter_ratio, a float num between [0, 1])" << endl;
        return -1;
    }
    string skel_file_path(argv[1]);
    string seeds_file_path(argv[2]);
    string patches_fle_path(argv[3]);
    int patch_num = atoi(argv[4]);
    int points_num_per_patch = atoi(argv[5]);
    float inter_ratio = atof(argv[6]);

    // cerr << "Debug: patch num: " << patch_num << endl;
    // prepare data container
    vector<Point3d> raw_skeleton_points;
    Graph skeleton_graph;

    // <seed, patch>
    // seed is Point4d (x, y, z, radius)
    vector<Point3d> output_points;
    vector<pair<Point4d, vector<Point3d>>> output_patches;

    read_skeleton_from_obj(skel_file_path, raw_skeleton_points, skeleton_graph);
    generate_skeletal_points(raw_skeleton_points, skeleton_graph, output_points, inter_ratio);

    generate_patches(raw_skeleton_points, skeleton_graph, output_points, output_patches, patch_num, points_num_per_patch);
    write_seeds_to_npts(output_patches, seeds_file_path);
    write_patches_to_npts(output_patches, patches_fle_path);
    return 0;
}
