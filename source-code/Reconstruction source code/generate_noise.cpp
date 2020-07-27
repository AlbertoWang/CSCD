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

using namespace std;

#define CONST_E		2.7182818284590452354	/* e */

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

struct BoundingBox {
    Point3d center;
    Vector3d extent;

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
        << kv.first.x << " " << kv.first.y << " " << kv.first.z << " " 
        << kv.second.x << " " << kv.second.y << " " << kv.second.z;
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
        max_point.x = std::max(kv.first.x, max_point.x);
        max_point.y = std::max(kv.first.y, max_point.y);
        max_point.z = std::max(kv.first.z, max_point.z);
        // ==
        min_point.x = std::min(kv.first.x, min_point.x);
        min_point.y = std::min(kv.first.y, min_point.y);
        min_point.z = std::min(kv.first.z, min_point.z);
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


void generate_noise(const vector<pair<Point3d, Vector3d>>& raw_points, vector<pair<Point3d, Vector3d>>& output_points, float noise_ratio, float noise_range_ratio) {
    // clamp noise_ratio and noise_range_ratio to [0, 1]
    noise_ratio = std::max(0.f, noise_ratio);
    noise_ratio = std::min(1.f, noise_ratio);
    noise_range_ratio = std::max(0.f, noise_range_ratio);
    noise_range_ratio = std::min(1.f, noise_range_ratio);

    // shuffle indices
    vector<int> indices(raw_points.size());
    for (int i = 0; i < raw_points.size(); i ++) indices[i] = i;
    std::random_shuffle(indices.begin(), indices.end());

    // compute bounding box
    const BoundingBox bbox = compute_bounding_box(raw_points);
    const int noise_point_cnt = std::floor(raw_points.size() * noise_ratio);
    const double noise_range = bbox.extent.norm() * 0.1 * noise_range_ratio; 
    const double gaussian_sigma = 0.5;
    auto gaussian_distrib = [=](double random_float) -> double { 
        return pow(CONST_E, 
                random_float * random_float / (2.0 * pow(gaussian_sigma, 2.0))
            );  };
    vector<double> gaussian_distribution_result;
    generate_distribution(noise_point_cnt, gaussian_distrib, 0.0, 1.0, gaussian_distribution_result);
    for (int i = 0; i < raw_points.size(); i ++) {
        int raw_point_index = indices[i];
        Point3d noise_point = raw_points[raw_point_index].first;
        const Vector3d& point_normal = raw_points[raw_point_index].second;
        if (i < noise_point_cnt) {
            double noise_length = gaussian_distribution_result[i] * noise_range;
            noise_point = noise_point + (point_normal * noise_length);
        }         
        output_points.push_back({noise_point, point_normal});
    }
}

int main(int argc, char** argv) {
    if (argc < 5) {
        cerr << "Usage:" << argv[0] << " *.npts(input)  *.npts(output) noise_ratio(0-1) noise_range(0-1)" << endl;
        return -1;
    }
    string npts_file_path(argv[1]);
    string output_file_path(argv[2]);
    float noise_ratio = atof(argv[3]);
    float noise_range_ratio = atof(argv[4]);

    // prepare data container
    vector<pair<Point3d, Vector3d>> raw_points;
    vector<pair<Point3d, Vector3d>> output_points;
    read_points_from_npts(npts_file_path, raw_points);
    generate_noise(raw_points, output_points, noise_ratio, noise_range_ratio);
    write_points_to_npts(output_points, output_file_path);
    return 0;
}
