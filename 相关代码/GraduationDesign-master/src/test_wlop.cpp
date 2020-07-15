#include <iostream>
#include <cmath>
#include <string>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "wlop_xyz.hpp"


using namespace std;
struct Point4d {
    double x;
    double y;
    double z;
    double r;
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

    double dot(const Point3d& ano) const{
        return this->x * ano.x + this->y * ano.y + this->z * ano.z;
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

int main() {
    srand(time(0));
    vector<Point3d> points;
    vector<Point3d> samples;
    double length = 10.0;
    int num = 90;
    for (int i = 0; i < num; i ++) {
        for (int j = 0; j < num; j ++) {
            points.emplace_back(i * length / num, j * length / num, 0.0);
        }
    }
    int sample_num = 300;
    for (int i = 0; i < sample_num; i ++) {
        int rand_index = rand() % points.size();
        Point3d point(points[rand_index]);
        samples.push_back(point);
    }
    using namespace zjl;
    cout << "Before resample: " << endl;
    for (int i = 0; i < 10; i ++) {
        cout << "(" << samples[i].x << ", " << samples[i].y << ", " << samples[i].z << ")" << endl;
    }
    Wlop<Point3d> wlop(points);
    wlop.resample(samples, 10);
    cout << endl << "After resample: " << endl;
    for (int i = 0; i < 10; i ++) {
        cout << "(" << samples[i].x << ", " << samples[i].y << ", " << samples[i].z << ")" << endl;
    }
    return 0;
}