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
    vector<Point3d> points;
    double length = 10.0;
    int num = 20;
    for (int i = 0; i < num; i ++) {
        for (int j = 0; j < num; j ++) {
            for (int k = 0; k < num; k ++) {
                points.emplace_back(i * length / num, j * length / num, k * length / num);
            }
        }
    }
    using namespace zjl;
    OctreeXYZ<Point3d> octree(points);
    vector<int> result;
    octree.build_index();
    //octree.search_neighbors_within_distance(Point3d(4.0, 4.0, 4.0), 1.2, result);
    octree.search_k_neighbors(10, Point3d(4.0, 4.0, 4.0), result);
    for (int result_index : result) {
        const Point3d& point = points[result_index];
        cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << endl;
    }
    // octree.search_k_neighbors(10, Point3d(0, 0, 0), result);
    // priority_queue<int, vector<int>, std::function<bool(int, int)>> queue([=](const int index1, const int index2){
    //     return index1 < index2;
    // });
    // queue.push(3);
    // queue.push(1);
    // queue.push(2);
    // while (!queue.empty()) {
    //     std::cout << queue.top() << " ";
    //     queue.pop();
    // }
    return 0;
}