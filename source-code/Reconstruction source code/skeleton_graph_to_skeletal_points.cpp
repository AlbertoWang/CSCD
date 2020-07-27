
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

struct Point3d {
    double x;
    double y;
    double z;

    Point3d(double px, double py, double pz):
    x(px), y(py), z(pz) {}

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

BoundingBox compute_bounding_box(const vector<pair<Point3d, Vector3d>>& raw_points) {
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
    for (const auto& point : raw_points) {
        max_point.x = std::max(point.first.x, max_point.x);
        max_point.y = std::max(point.first.y, max_point.y);
        max_point.z = std::max(point.first.z, max_point.z);
        // ==
        min_point.x = std::min(point.first.x, min_point.x);
        min_point.y = std::min(point.first.y, min_point.y);
        min_point.z = std::min(point.first.z, min_point.z);
    }
    center = (max_point + min_point) * 0.5;
    extent = (max_point - min_point) * 0.5;
    return {center, extent};
}

/**
 * D should be a pdf(passed as a functor or a lambda), whose value f(x) range from 0 to 1.
 * the generated distribution will range from [val_lower_bound, val_upper_bound], subjecting to D(x)
 */
template<typename D>
void generate_distribution(const int sample_num,  D&& distri, const double val_lower_bound, const double val_upper_bound, vector<double>& target_distribution) {
    if (val_lower_bound >= val_upper_bound) {
        cerr << "Error: val_lower_bound >= val_upper_bound! " << endl;
        return;
    }

    target_distribution.clear();
    std::uniform_real_distribution<double> uniform_distrib(val_lower_bound, val_upper_bound);
    std::uniform_real_distribution<double> uniform_distrib_01(0.0, 1.0);
    std::default_random_engine random_engine;
    int cnt_generated = 0;
    while (target_distribution.size() < sample_num) {
        double real_random_float = uniform_distrib(random_engine);
        double real_random_float_p = uniform_distrib_01(random_engine);
        double pdf_val = distri(real_random_float);
        if (real_random_float_p < pdf_val) {
            target_distribution.push_back(real_random_float);
        }
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
}

int main(int argc, char** argv) {
    if (argc < 4) {
        cerr << "Usage:" << argv[0] << " *.obj(input, skeleton graph)  *.npts(output) inter_ratio(0-1) " << endl;
        return -1;
    }
    string skel_file_path(argv[1]);
    string output_file_path(argv[2]);
    float inter_ratio = atof(argv[3]);

    // prepare data container
    vector<Point3d> raw_skeleton_points;
    Graph skeleton_graph;

    vector<Point3d> output_points;

    read_skeleton_from_obj(skel_file_path, raw_skeleton_points, skeleton_graph);
    generate_skeletal_points(raw_skeleton_points, skeleton_graph, output_points, inter_ratio);
    write_points_to_npts(output_points, output_file_path);
    return 0;

}
