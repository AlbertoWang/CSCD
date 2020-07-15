#include <iostream>
#include <cmath>
#include <string>
#include <limits>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <algorithm>
#include <unordered_map>

#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace std;
// using namespace Eigen;

#define CONST_E		2.7182818284590452354	/* e */
#define CONST_PI	3.1415926	            /* pi */


using Eigen::Vector3d;
using Eigen::Matrix4d;
using Point3d = Eigen::Vector3d;


struct BoundingBox {
    Point3d center;
    Vector3d extent;
};

struct OcclusionBox {
    Point3d center;
    Vector3d extent;
    Vector3d rotation;
    Matrix4d rotation_matrix;
    Matrix4d rotation_matrix_inv;

    OcclusionBox(const Point3d& center, const Vector3d& extent, const Vector3d& rotation):
    center(center), extent(extent), rotation(rotation) {
        Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::Translation3d(center * -1.0));
        rotation_matrix_inv = t.matrix();   
        // rotation_matrix_inv = rotation_matrix.inverse();
        // cout << "matrix: " << rotation_matrix;
        // cout << "center\n" << center << endl;
        // cout << "extent\n" << extent << endl;
        // cout << "rotation\n " << rotation << endl;
        // cout << "matrix_inv: " << rotation_matrix_inv << endl;
        t
        .rotate(Eigen::AngleAxisd(rotation(0), Vector3d::UnitX()))
        .rotate(Eigen::AngleAxisd(rotation(1), Vector3d::UnitY()))
        .rotate(Eigen::AngleAxisd(rotation(2), Vector3d::UnitZ()))
        ;
        rotation_matrix_inv = t.matrix();   
        // rotation_matrix_inv = rotation_matrix.inverse();
        // cout << "matrix: " << rotation_matrix;
        cout << "matrix_inv: " << rotation_matrix_inv << endl;
        //Eigen::Translation3d move_to_origin(- center(0), - center(1), - center(2));
        //Eigen::Translation3d restore_from_origin(center(0), center(1), center(2));
    }

    bool contains(const Vector3d& point) const {
        Vector3d point_inv = (rotation_matrix_inv * point.homogeneous()).hnormalized();
        // cout << "point: " << point << ", point_inv: " << point_inv << endl;
        return 
        point_inv(0) <= extent(0) && 
        point_inv(1) <= extent(1) && 
        point_inv(2) <= extent(2) && 
        point_inv(0) >= - extent(0) && 
        point_inv(1) >= - extent(1) && 
        point_inv(2) >= - extent(2);
    }
};

void read_points_from_npts(const string& input_file, vector<pair<Point3d, Vector3d>> &points) {
    ifstream ifs(input_file);
    if (!ifs.is_open()) {
        cerr << "Error: can't open " << input_file << endl;
        return;
    }
    points.clear();
    string line;
    double x = 0, y = 0, z = 0, nx = 0, ny = 0, nz = 0;
    while (std::getline(ifs, line)) {
        if (line.find("nan") != line.npos) continue;
        istringstream iss(line);
        if (!(iss >> x >> y >> z >> nx >> ny >> nz)) {
            cerr << "Error: failed to read six float number from line: " << line << endl;
            break;
        }
        points.push_back({{x, y, z}, {nx, ny, nz}});
    }
}

void write_points_to_npts(const vector<pair<Point3d, Vector3d>>& points, const string& file_name) {
    ofstream ofs(file_name);
    if (!ofs.is_open()) {
        cerr << "Error: failed to open file " << file_name << endl;
        return;
    }
    bool first_line = true;
    for (const auto& kv : points) {
        if (first_line) first_line = false; else ofs << endl;
        ofs 
        << kv.first(0) << " " << kv.first(1) << " " << kv.first(2) << " " 
        << kv.second(0) << " " << kv.second(1) << " " << kv.second(2);
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
    for (const auto& kv : raw_points) {
        max_point(0) = std::max(kv.first(0), max_point(0));
        max_point(1) = std::max(kv.first(1), max_point(1));
        max_point(2) = std::max(kv.first(2), max_point(2));
        // ==
        min_point(0) = std::min(kv.first(0), min_point(0));
        min_point(1) = std::min(kv.first(1), min_point(1));
        min_point(2) = std::min(kv.first(2), min_point(2));
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


void generate_occlusion(const vector<pair<Point3d, Vector3d>>& raw_points, vector<pair<Point3d, Vector3d>>& output_points, int occlusion_cnt, float occlusion_range_ratio) {
    // clamp occlusion_cnt and occlusion_range_ratio to [0, 1]
    occlusion_cnt = std::max(0, occlusion_cnt);
    occlusion_cnt = std::min(5, occlusion_cnt);
    occlusion_range_ratio = std::max(0.f, occlusion_range_ratio);
    occlusion_range_ratio = std::min(1.f, occlusion_range_ratio);

    // shuffle indices
    vector<int> indices(raw_points.size());
    for (int i = 0; i < raw_points.size(); i ++) indices[i] = i;
    std::random_shuffle(indices.begin(), indices.end());

    // generate occlusion box
    vector<OcclusionBox> occlusion_boxs;
    std::uniform_real_distribution<double> uniform_distrib_01(0.0, 1.0);
    std::default_random_engine random_engine;
    random_engine.seed(chrono::system_clock::now().time_since_epoch().count());
    const BoundingBox bbox = compute_bounding_box(raw_points);
    const double occlusion_range = bbox.extent.norm() * 0.2 * occlusion_range_ratio; 
    for (int i = 0; i < occlusion_cnt; i ++) {
        Vector3d point_center = raw_points[indices[i]].first;
        
        Vector3d box_diag_extent(occlusion_range, occlusion_range, occlusion_range * 0.2);
        Vector3d random_rotation(uniform_distrib_01(random_engine) * CONST_PI, uniform_distrib_01(random_engine) * CONST_PI, uniform_distrib_01(random_engine) * CONST_PI);
        // fixed-box
        int j = 0;
        for (j = 0; j < raw_points.size(); j ++) {
            if (raw_points[j].first(1) < bbox.center(1)) break; 
        }
        point_center = raw_points[j].first;
        box_diag_extent = {occlusion_range, occlusion_range * 0.3, occlusion_range};
        random_rotation = {0, 0, 0};
        // fixed-box
        OcclusionBox occlusion_box = {point_center, box_diag_extent, random_rotation};
        occlusion_boxs.push_back(occlusion_box);
    }

    int occlusion_point_num = 0;
    for (int i = 0; i < raw_points.size(); i ++) {
        Point3d result_point = raw_points[i].first;
        const Vector3d& point_normal = raw_points[i].second;
        bool is_removed = false;
        for (const auto& box : occlusion_boxs) {
            if (box.contains(result_point)) {
                is_removed = true;
                break;
            }
        }
        if (is_removed) {
            occlusion_point_num += 1;
            continue;
        }
        output_points.push_back({result_point, point_normal});
    }
    cout << "Remove " << occlusion_point_num << " points from the raw point cloud." << endl;
}

int main(int argc, char** argv) {
    if (argc < 5) {
        cerr << "Usage:" << argv[0] << " *.npts(input)  *.npts(output) occlusion_cnt(0-5) occlusion_range(0-1)" << endl;
        return -1;
    }
    string npts_file_path(argv[1]);
    string output_file_path(argv[2]);
    int occlusion_cnt = atof(argv[3]);
    float occlusion_range_ratio = atof(argv[4]);

    // prepare data container
    vector<pair<Point3d, Vector3d>> raw_points;
    vector<pair<Point3d, Vector3d>> output_points;
    read_points_from_npts(npts_file_path, raw_points);
    generate_occlusion(raw_points, output_points, occlusion_cnt, occlusion_range_ratio);
    write_points_to_npts(output_points, output_file_path);
    return 0;
}
