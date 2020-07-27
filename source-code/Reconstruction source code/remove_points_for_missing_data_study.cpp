#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include "point_xyz.hpp"
#include "io_xyz.hpp"

using namespace std;

struct Skeleton {
    std::vector<Point3d> nodes;
    std::vector<Edge> edges;
};

struct Box {
    Point3d center;
    Vector3d extent;

    Box(): center({0, 0, 0}), extent({0, 0, 0}){}

    bool contains(const Point3d& p) const {
        return 
            p.x >= center.x - extent.x && p.x <= center.x + extent.x && 
            p.y >= center.y - extent.y && p.y <= center.y + extent.y && 
            p.z >= center.z - extent.z && p.z <= center.z + extent.z  
        ;
    }
};

/**
 * @return: the bounding box of the removed points
 */
Box remove_points_from_pc(const std::vector<Point3d>& intact_points, std::vector<Point3d>& incomplete_points, float ratio) {
    Box box;
    // *********************
    // mve-scan-2
    // ===
    // ratio: 0.053 points: 3660
    // box.center = {0.0, -0.5, 0.0}; 
    // box.extent = {0.35, 0.09, 0.2};
    // incomplete_points.clear();
    // ===
    // ratio: 0.1048 points: 7140
    // box.center = {0.0, -0.5, 0.0}; 
    // box.extent = {0.4, 0.15, 0.3};
    // ===
    // ratio: 0.1518 points: 10339 
    // box.center = {0.0, -0.45, 0.0}; 
    // box.extent = {0.4, 0.163, 0.3};
    // ===
    // ratio: 0.198 points: 13497 
    // box.center = {0.0, -0.45, 0.0}; 
    // box.extent = {0.45, 0.213, 0.4};
    // ===
    // ratio: 0.248 points: 16924
    // box.center = {0.0, -0.40, 0.0}; 
    // box.extent = {0.45, 0.242, 0.4};
    //
    // *********************
    // mve-scan-5
    // ===
    // ratio: 0.053 points: 1858
    // box.center = {0.0, -0.40, 0.0}; 
    // box.extent = {0.35, 0.08, 0.2};
    // ===
    // ratio: 0.1024 points: 3593
    // box.center = {0.0, -0.35, 0.0}; 
    // box.extent = {0.35, 0.13, 0.2};
    // ===
    // ratio: 0.149 points: 5223
    // box.center = {0.0, -0.35, 0.0}; 
    // box.extent = {0.35, 0.13, 0.2};
    // ===
    // ratio: 0.2026 points: 7107
    // box.center = {0.0, -0.35, 0.0}; 
    // box.extent = {0.40, 0.25, 0.3};
    // ===
    // ratio: 0.248 points: 8724
    // box.center = {0.0, -0.35, 0.0}; 
    // box.extent = {0.40, 0.313, 0.3};
    // ===
    //
    // *********************
    // mve-scan-8
    // ===
    // ratio: 0.0497 points: 1612
    // box.center = {0.0, -0.15, 0.0}; 
    // box.extent = {0.40, 0.02, 0.3};
    // ===
    // ratio: 0.101 points: 3282
    // box.center = {0.0, -0.15, 0.0}; 
    // box.extent = {0.40, 0.042, 0.3};
    // ===
    // ratio: 0.153 points: 4960
    // box.center = {0.0, -0.15, 0.0}; 
    // box.extent = {0.40, 0.042, 0.3};
    // ===
    // ratio: 0.2037 points: 6601
    // box.center = {0.0, -0.15, 0.0}; 
    // box.extent = {0.40, 0.085, 0.3};
    // ===
    // ratio: 0.2524 points: 8178
    // box.center = {0.0, -0.15, 0.0}; 
    // box.extent = {0.40, 0.105, 0.3};
    // ===
    //
    // #####################
    //
    // *********************
    // raw-scan-4
    // ===
    // ratio: 0.053 points: 1814
    // box.center = {0.0, -0.18, 0.2}; 
    // box.extent = {0.40, 0.055, 0.3};
    // ===
    // ratio: 0.1034 points: 3491
    // box.center = {0.0, -0.18, 0.2}; 
    // box.extent = {0.5, 0.095, 0.45};
    // ===
    // ratio: 0.1515 points: 5112
    // box.center = {0.0, -0.18, 0.2}; 
    // box.extent = {0.5, 0.135, 0.45};
    // ===
    // ratio: 0.2010 points: 6782
    // box.center = {0.0, -0.14, 0.2}; 
    // box.extent = {0.5, 0.175, 0.45};
    // ===
    // ratio: 0.253 points: 8564
    // box.center = {0.0, -0.14, 0.2}; 
    // box.extent = {0.55, 0.219, 0.48};
    // ===
    //
    // *********************
    // raw-scan-10
    // ===
    // ratio: 0.053 points: 288
    // box.center = {0.0, -0.5, 0.2}; 
    // box.extent = {0.35, 0.070, 0.38};
    // ===
    // ratio: 0.10374 points: 1034
    // box.center = {0.0, -0.5, 0.2}; 
    // box.extent = {0.35, 0.070, 0.38};
    // ===
    // ratio: 0.151 points: 1511
    // box.center = {0.0, -0.4, 0.2}; 
    // box.extent = {0.39, 0.168, 0.42};
    // ===
    // ratio: 0.1986 points: 1980 
    // box.center = {0.0, -0.4, 0.2}; 
    // box.extent = {0.39, 0.225, 0.42};
    // ===
    // ratio: 0.251 points: 2507
    // box.center = {0.0, -0.4, 0.2}; 
    // box.extent = {0.39, 0.290, 0.42};
    // ===
    //
    // *********************
    // raw-scan-17
    // ===
    // ratio: 0.050 points: 488
    // box.center = {0.0, -0.2, 0.14}; 
    // box.extent = {0.35, 0.040, 0.3};
    // ===
    // ratio: 0.1046 points: 1015
    // box.center = {0.0, -0.2, 0.14}; 
    // box.extent = {0.20, 0.098, 0.2};
    // ===
    // ratio: 0.149 points: 1451
    // box.center = {0.0, -0.2, 0.14}; 
    // box.extent = {0.20, 0.145, 0.2};
    // ===
    // ratio: 0.20 points: 1951
    // box.center = {0.0, -0.2, 0.14}; 
    // box.extent = {0.30, 0.170, 0.3};
    // ===
    // ratio: 0.253 points: 2454
    // box.center = {0.0, -0.2, 0.14}; 
    // box.extent = {0.30, 0.210, 0.3};
    // ===
    box.center = {0.0, -0.2, 0.14}; 
    box.extent = {0.30, 0.210, 0.3};
    incomplete_points.clear();

    int remove_point_cnt = 0;
    for (auto p : intact_points) {
        if (box.contains(p)) {
            remove_point_cnt += 1;
        } else {
            incomplete_points.push_back(p);
        }
    }
    float real_ratio = remove_point_cnt * 1.0 / intact_points.size(); 
    cout << "Remove " << remove_point_cnt << " points, the ratio is about " << real_ratio << std::endl;
    return box;
}

void remove_nodes_from_skel(const Skeleton& intact_skeleton, Skeleton& incomplete_skeleton, const Box& box) {
    incomplete_skeleton.nodes.clear();
    incomplete_skeleton.edges.clear();

    // intact_skeleton indices --> incomplete_skeleton
    unordered_map<int, int> indices_map;
    
    int remove_node_cnt = 0;
    int remove_edge_cnt = 0;
    for (int i = 0; i < intact_skeleton.nodes.size(); i ++) {
        auto& node = intact_skeleton.nodes[i]; 
        if (! box.contains(node)) {
            indices_map[i] = incomplete_skeleton.nodes.size();
            incomplete_skeleton.nodes.push_back(node);
        } else {
            remove_node_cnt += 1;
        }
    }
    for (Edge edge : intact_skeleton.edges) {
        if (indices_map.find(edge.vi) == indices_map.end() ||
            indices_map.find(edge.vj) == indices_map.end() ) {
            remove_edge_cnt += 1;
            continue;
        }
        edge.vi = indices_map[edge.vi];
        edge.vj = indices_map[edge.vj];
        incomplete_skeleton.edges.push_back(edge);
    }
    std::cout << "Remove " << remove_node_cnt << " nodes, " << remove_edge_cnt << " edges from the skeleton " << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0] << "\n\txx.npts(intact point cloud) \n\txx.obj(intact skeleton) \n\txx.npts(output incomplete point cloud) \n\txx.obj(output incomplete skeleton) \n\t0.x(float number, the percentage of points to be removed) " << std::endl;
        return -1;
    }
    float ratio = atof(argv[5]);
    if (ratio < 0.0 || ratio > 1.0) {
        cerr << "Invalid missing percentage: " << ratio << std::endl;
        return -1;
    }
    Skeleton intact_skeleton;
    Skeleton incomplete_skeleton;
    std::vector<Point3d> intact_points;
    std::vector<Point3d> incomplete_points;
    zjl::IO::read_point_cloud_from_npts_without_normals(argv[1], intact_points);
    zjl::IO::read_graph_from_obj(argv[2], intact_skeleton.nodes, intact_skeleton.edges);

    if (intact_points.empty() || intact_skeleton.nodes.empty()) {
        cerr << "Empty" << std::endl;
        return -1;
    }

    // ===
    auto box = remove_points_from_pc(intact_points, incomplete_points, ratio);
    remove_nodes_from_skel(intact_skeleton, incomplete_skeleton, box);
    // ===

    zjl::IO::write_point_cloud_to_npts_without_normals(argv[3], incomplete_points);
    zjl::IO::write_graph_to_obj(argv[4], incomplete_skeleton.nodes, incomplete_skeleton.edges);
    return 0;
}
