#include <iostream>
#include <vector>
#include <algorithm>
#include "point_xyz.hpp"
#include "io_xyz.hpp"

using namespace std;

struct TriMesh {
    std::vector<Point3d> vertices;
    std::vector<Triangle> faces;
};

struct Skeleton {
    std::vector<Point3d> skeleton_points;
    std::vector<Edge> skeleton_edges;
};

void normalize(std::vector<Point3d>& mesh_points, std::vector<Point3d>& skel_points) {
    if (mesh_points.empty()) {
        cerr << "Error: no points" << endl;
        exit(-1);
    }
    const double LD = std::numeric_limits<double>::lowest();
    const double MD = std::numeric_limits<double>::max();
    Point3d max_point(LD, LD, LD);
    Point3d min_point(MD, MD, MD);
    for (const auto& kv : mesh_points) {
        max_point.x = std::max(kv.x, max_point.x);
        max_point.y = std::max(kv.y, max_point.y);
        max_point.z = std::max(kv.z, max_point.z);
            // ==
        min_point.x = std::min(kv.x, min_point.x);
        min_point.y = std::min(kv.y, min_point.y);
        min_point.z = std::min(kv.z, min_point.z);
    }
    
    auto center = (max_point + min_point) * 0.5;
    auto diagonal = (max_point - min_point) * 0.5;

    double scaling = 1.0 / std::max({diagonal.x, diagonal.y, diagonal.z});
    for (auto& point : mesh_points) {
        point = point - center;
        point = point * scaling;
    }
    for (auto& point : skel_points) {
        point = point - center;
        point = point * scaling;
    }
}

int main(int argc, char** argv) {
    if (argc < 5) {
        cout << "Usage: " << argv[0] << " xx.ply(mesh) xxx.obj(skeleton) xxx.ply(normalized mesh) xxx.obj(normalized skel" << endl;
        return -1;
    }
    TriMesh mesh;
    Skeleton skeleton;
    zjl::IO::read_trimesh_from_ply(argv[1], mesh.vertices, mesh.faces);
    zjl::IO::read_graph_from_obj(argv[2], skeleton.skeleton_points, skeleton.skeleton_edges);
    normalize(mesh.vertices, skeleton.skeleton_points);
    zjl::IO::write_trimesh_to_ply_without_normals(argv[3], mesh.vertices, mesh.faces);
    zjl::IO::write_graph_to_obj(argv[4], skeleton.skeleton_points, skeleton.skeleton_edges);
    return 0;
}