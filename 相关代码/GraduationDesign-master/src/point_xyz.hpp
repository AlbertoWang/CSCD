#pragma once
#include <cmath>
#include <array>

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

    Point3d operator/ (const float r) const {
        return {this->x / r, this->y / r, this->z / r};
    }
};

struct Edge {
    int vi;
    int vj;
    Edge(int i, int j) :
        vi(i),
        vj(j)
    {}
};

struct Triangle {
    int vi;
    int vj;
    int vk;
    Triangle(int i, int j, int k):vi(i), vj(j), vk(k){}
};

using Vector3d = Point3d;