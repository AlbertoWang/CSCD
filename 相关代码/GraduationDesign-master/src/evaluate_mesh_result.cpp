#include <iostream>
#include <cstdlib>
#include <cstdio>
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
#include "point_xyz.hpp"
#include "io_xyz.hpp"
#include "octree_xyz.hpp"

using namespace std;

struct TriangleMesh {
    std::vector<Point3d> vertices;
    std::vector<Triangle> faces;
};

void compute_nearest_vertex_pairs(const TriangleMesh& mesh1, const TriangleMesh& mesh2, unordered_map<const Point3d*, pair<const Point3d*, double> >& s1_to_s2, unordered_map<const Point3d*, pair<const Point3d*, double> >& s2_to_s1) {
    s1_to_s2.clear();
    s2_to_s1.clear();

    zjl::OctreeXYZ<Point3d> m2_octree(mesh2.vertices);
    m2_octree.build_index();
    zjl::OctreeXYZ<Point3d> m1_octree(mesh1.vertices);
    m1_octree.build_index();

    std::vector<int> nearest_three_vertex_indices;
    for (const auto& vertex : mesh1.vertices) {
        double nearest_distance = std::numeric_limits<double>::max();
        const Point3d* nearest_vertex_ptr = nullptr;
        m2_octree.search_k_neighbors(3, vertex, nearest_three_vertex_indices);

        for (const auto& vertex2_index : nearest_three_vertex_indices) {
            const auto& vertex2 = mesh2.vertices[vertex2_index];
            double distance = (vertex - vertex2).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_vertex_ptr = & vertex2;
            } 
        }
        s1_to_s2[&vertex] = {nearest_vertex_ptr, nearest_distance};
    }
    for (const auto& vertex : mesh2.vertices) {
        double nearest_distance = std::numeric_limits<double>::max();
        const Point3d* nearest_vertex_ptr = nullptr;

        m1_octree.search_k_neighbors(3, vertex, nearest_three_vertex_indices);
        for (const auto& vertex1_index : nearest_three_vertex_indices) {
            const auto& vertex1 = mesh1.vertices[vertex1_index];
            double distance = (vertex - vertex1).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_vertex_ptr = & vertex1;
            } 
        }
        s2_to_s1[&vertex] = {nearest_vertex_ptr, nearest_distance};
    }
}

double compute_hausdorff_distance(const TriangleMesh& mesh, const TriangleMesh& gt) {
    double distance = 0.0;
    unordered_map<const Point3d*, pair<const Point3d*, double> > s1_to_s2; unordered_map<const Point3d*, pair<const Point3d*, double> > s2_to_s1;
    compute_nearest_vertex_pairs(mesh, gt, s1_to_s2, s2_to_s1); 
    for (auto& pp : s1_to_s2) {
        distance = std::max(distance, pp.second.second);
    }
    for (auto& pp : s2_to_s1) {
        distance = std::max(distance, pp.second.second);
    }
    return distance;
}

double compute_chamfer_distance(const TriangleMesh& mesh, const TriangleMesh& gt) {
    double distance = 0.0;
    unordered_map<const Point3d*, pair<const Point3d*, double> > s1_to_s2; unordered_map<const Point3d*, pair<const Point3d*, double> > s2_to_s1;
    compute_nearest_vertex_pairs(mesh, gt, s1_to_s2, s2_to_s1); 
    for (auto& pp : s1_to_s2) {
        distance += pow(pp.second.second, 2.0);
    }
    distance /= s1_to_s2.size();
    double distance2 = 0.0;
    for (auto& pp : s2_to_s1) {
        distance2 += pow(pp.second.second, 2.0);
    }
    distance2 /= s2_to_s1.size();
    distance += distance2;
    //distance /= (mesh.vertices.size() + gt.vertices.size());
    return distance;

}


double compute_mean_distance(const TriangleMesh& mesh, const TriangleMesh& gt) {
    double distance = 0.0;
    unordered_map<const Point3d*, pair<const Point3d*, double> > s1_to_s2; unordered_map<const Point3d*, pair<const Point3d*, double> > s2_to_s1;
    compute_nearest_vertex_pairs(mesh, gt, s1_to_s2, s2_to_s1); 
    for (auto& pp : s1_to_s2) {
        distance += pp.second.second;
    }
    for (auto& pp : s2_to_s1) {
        distance += pp.second.second;
    }
    distance /= (mesh.vertices.size() + gt.vertices.size());
    return distance;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " xxx.ply(output by mesheton_based_tree_generation.exe) xxx.ply(ground truth tree mesh)" << endl;
        return -1;
    }
    // cout << "Hello world" << endl;
    TriangleMesh result;
    TriangleMesh gt;
    zjl::IO::read_trimesh_from_ply(argv[1], result.vertices, result.faces);
    zjl::IO::read_trimesh_from_ply(argv[2], gt.vertices, gt.faces);

    //double hausdorff_dis = compute_hausdorff_distance(result, gt);
    double mean_dis = compute_mean_distance(result, gt);
    //double chamfer_dis = compute_chamfer_distance(result, gt);

    cout 
        //<< argv[1]          << '\t' 
        //<< hausdorff_dis    << '\t' 
        //<< chamfer_dis      
        <<  std::endl;

    FILE *fp = fopen("./evaluation.log", "a+");
    if (fp == nullptr) {
        cout << "Fail to open file" << endl;
        return 0;
    }
    fprintf(fp, "%s\t%.6f\n", argv[1], mean_dis);
    return 0;
}