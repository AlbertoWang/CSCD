#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "io_xyz.hpp"

using namespace std;

struct Edge {
    int vi;
    int vj;

};

struct Triangle {
    int vi;
    int vj;
    int vk;
    Triangle(int i, int j, int k):vi(i), vj(j), vk(k){}
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

void test_write_off() {
    vector<Point3d> vertices;
    vector<Triangle> tris;
    // test write off
    vertices.clear();
    tris.clear();
    vertices.emplace_back(1, 1, -1);
    vertices.emplace_back(1, -1, -1);
    vertices.emplace_back(1, 1, 1);
    vertices.emplace_back(1, -1, 1);
    vertices.emplace_back(-1, 1, -1);
    vertices.emplace_back(-1, -1, -1);
    vertices.emplace_back(-1, 1, 1);
    vertices.emplace_back(-1, -1, 1);
    tris.emplace_back(5, 3, 1);
    tris.emplace_back(3, 8, 4);
    tris.emplace_back(7, 6, 8);
    tris.emplace_back(2, 8, 6);
    tris.emplace_back(1, 4, 2);
    tris.emplace_back(5, 2, 6);
    tris.emplace_back(5, 7, 3);
    tris.emplace_back(3, 7, 8);
    tris.emplace_back(7, 5, 6);
    tris.emplace_back(2, 4, 8);
    tris.emplace_back(1, 3, 4);
    tris.emplace_back(5, 1, 2);
    zjl::IO::write_trimesh_to_off<Point3d, Triangle>("./test.off", vertices, tris);

}

void test_read_ply() {
    vector<Point3d> vertices;
    vector<Triangle> tris;
    zjl::IO::read_trimesh_from_ply<Point3d, Triangle>("./test_case_ply.ply", vertices, tris);
}

int main(int argc, char** argv) {
    test_read_ply();
    return 0;
}

