#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <unordered_map>
#include "point_xyz.hpp"
#include "octree_xyz.hpp"
#include "io_xyz.hpp"

using namespace std;

double math_e = 2.71828;
double taper_threshold = 10000.;

struct Skeleton {
    std::vector<Point3d> skeleton_points;
    std::vector<Edge> skeleton_edges;
};

void write_radius_info_to_ri(const std::string& filepath, const unordered_map<int, double>& radius_info) {
    ofstream ofs(filepath);
    if (!ofs.is_open()) {
        std::cerr << "Error: cannot open " << filepath << std::endl;
        return;
    }
    for (const auto& kv : radius_info) {
        ofs << kv.first << " " << kv.second << std::endl;
    }
    std::cout << "Succeed write " << radius_info.size() << " kv-pairs into " << filepath << std::endl;
}

struct SkeletonNodeInfo {

    /* the index of the parent node in the skeleton graph, the index of the root node is that of itself*/
    int parent_node_index;

    /* the indices of the children nodes */
    std::vector<int> children;

    /* normalized to a range of 0 to 1 */
    float height; 

    /* indicates the real average distance from each points to the skeleton, 0 - 2 */
    float average_distance; 

    /* the radius of the branch in the generated mesh */
    float branch_radius; 

    /* if the degree of this node is larger than 2, false by default */
    bool is_branching; 

    /* if the degree of this node is 1, false by default */
    bool is_end;

    bool is_root; 

    /* How far is this node from the nearest terminal node(how many nodes) */
    int terminal_distance;

    /* How far is this node from the furthest terminal node(Euclidean distance) */
    double furthest_terminal_distance;

    double mass_supporting;

    Point3d tangent_vector;

    SkeletonNodeInfo():
        parent_node_index(-1), 
        height(0.0), // redundant info
        average_distance(0.0), 
        branch_radius(1.0), 
        is_branching(false), // redundant info
        // is_end(false),  // redundant info
        is_root(false),
        terminal_distance(-1),
        furthest_terminal_distance(-1.0),
        tangent_vector({0.0f, 1.0f, 0.0f})
        //mAuxXVector(0.0f, 0.0f, 0.0f)
    {}
};

std::pair<double, double> stat_furthest_distance(int curr_node_id, const Skeleton& skel, unordered_map<int, SkeletonNodeInfo>& skeleton_info) {
    if (curr_node_id < 0) {
        std::cerr << "Error: Something wrong." << std::endl;
        return {0.0, 0.0};
    }
    int children_size = skeleton_info[curr_node_id].children.size();
    if (children_size == 0) {
        skeleton_info[curr_node_id].furthest_terminal_distance = 0.0f;
        skeleton_info[curr_node_id].mass_supporting = 0.0f;
        return {0.0f, 0.0f};
    } else if (children_size == 1) {
        int next_node_id = skeleton_info[curr_node_id].children.front();
        auto nfd_ms_pair = stat_furthest_distance(next_node_id, skel, skeleton_info);
        double next_furthest_distance = nfd_ms_pair.first;
        double next_mass_supporting = nfd_ms_pair.second;
        const auto& curr_node = skel.skeleton_points[curr_node_id]; 
        const auto& next_node = skel.skeleton_points[next_node_id]; 
        double adj_node_distance = (curr_node - next_node).norm();
        double furthest_distance = adj_node_distance + next_furthest_distance;
        double mass_supporting = adj_node_distance + next_mass_supporting;
        // ===
        skeleton_info[curr_node_id].furthest_terminal_distance = furthest_distance;
        skeleton_info[curr_node_id].mass_supporting = mass_supporting;
        return {furthest_distance, mass_supporting};
    } else {
        const auto& curr_node = skel.skeleton_points[curr_node_id]; 
        double furthest_distance = 0.0f;
        double mass_supporting = 0.0f;
        for (int next_node_id : skeleton_info[curr_node_id].children) {
            auto nfd_ms_pair = stat_furthest_distance(next_node_id, skel, skeleton_info);
            double next_furthest_distance = nfd_ms_pair.first;
            double next_mass_supporting = nfd_ms_pair.second;
            const auto& next_node = skel.skeleton_points[next_node_id]; 
            double adj_node_distance = (curr_node - next_node).norm();
            furthest_distance = std::max(adj_node_distance + next_furthest_distance, furthest_distance);
            mass_supporting += (adj_node_distance + next_mass_supporting);
        }
        skeleton_info[curr_node_id].furthest_terminal_distance = furthest_distance;
        skeleton_info[curr_node_id].mass_supporting = mass_supporting;
        return {furthest_distance, mass_supporting};
    }
}

int analyze_skeleton(const Skeleton& skeleton, unordered_map<int, SkeletonNodeInfo>& skeleton_info) {

    skeleton_info.clear();

    // find root point
    int rootNodeIndex = 0;
    for (int rowIndex = 0; rowIndex < skeleton.skeleton_points.size(); rowIndex++) {
        if (skeleton.skeleton_points.at(rowIndex).y < skeleton.skeleton_points.at(rootNodeIndex).y) {
            rootNodeIndex = rowIndex;
        }
    }
    skeleton_info[rootNodeIndex].is_root = true;
    skeleton_info[rootNodeIndex].height = 0.0f;
    skeleton_info[rootNodeIndex].parent_node_index = rootNodeIndex;

    // build a simple data structure that represents for a undirected graph
    double aver_edge_length = 0.0;
    unordered_map<int, vector<int> > edge_map;
    for (int rowIndex = 0; rowIndex < skeleton.skeleton_edges.size(); rowIndex++) {
        int va = skeleton.skeleton_edges.at(rowIndex).vi;
        int vb = skeleton.skeleton_edges.at(rowIndex).vj;
        const auto& vert_a = skeleton.skeleton_points.at(va);
        const auto& vert_b = skeleton.skeleton_points.at(vb);
        aver_edge_length += (vert_a - vert_b).norm();
        if (find(edge_map[va].begin(), edge_map[va].end(), vb) == edge_map[va].end()) {
            edge_map[va].push_back(vb);
        }
        if (find(edge_map[vb].begin(), edge_map[vb].end(), va) == edge_map[vb].end()) {
            edge_map[vb].push_back(va);
        }
    }
    aver_edge_length /= skeleton.skeleton_edges.size();
    taper_threshold = aver_edge_length * 18;

    // check if contains cycle
    bool contains_cycle = false;
    vector<bool> visitMap(skeleton.skeleton_points.size());
    vector<pair<int, int>> visitQueue;;; visitQueue.push_back({rootNodeIndex, -1});
    for (size_t i = 0; i < visitMap.size(); i++) visitMap[i] = false;
    for (size_t i = 0; i < visitQueue.size(); i++) {
        int currNodeIndex = visitQueue[i].first;
        if (visitMap[currNodeIndex]) { 
            contains_cycle = true; 
            std::cout << "Cycle is located at (" << skeleton.skeleton_points.at(currNodeIndex).x
                << ", " << skeleton.skeleton_points.at(currNodeIndex).y
                << ", " << skeleton.skeleton_points.at(currNodeIndex).z << ")\n";
            break; 
        }
        visitMap[currNodeIndex] = true;
        for (int adj : edge_map[currNodeIndex]) {
            if (adj == visitQueue[i].second) continue;;; // parent
            visitQueue.push_back({ adj, currNodeIndex });
        }
    }
    if (contains_cycle) {
        std::cerr << "Cycle is found in the skeleton.... stop analyzing skeleton" << std::endl;
        return -1;
    }

    std::cout << "Start to mark parents." << std::endl;
    // mark parents
    vector<int> nodeIndicesQueue;
    nodeIndicesQueue.reserve(static_cast<int>(skeleton.skeleton_points.size() * 1.5));
    nodeIndicesQueue.push_back(rootNodeIndex);
    int currPointer = 0;
    float maxHeight = 0.0;
    while (currPointer < nodeIndicesQueue.size()) {
        int currNodeIndex = nodeIndicesQueue[currPointer];
        Point3d childrenMedian(0, 0, 0);
        Point3d parentNode = skeleton.skeleton_points.at(skeleton_info[currNodeIndex].parent_node_index);
        int childCnt = 0;
        for (int adjacentNodeIndex : edge_map.at(currNodeIndex)) {
            if (adjacentNodeIndex == skeleton_info[currNodeIndex].parent_node_index) continue;;;

            nodeIndicesQueue.push_back(adjacentNodeIndex);
            skeleton_info[adjacentNodeIndex].parent_node_index = currNodeIndex;
            skeleton_info[adjacentNodeIndex].height = skeleton_info[currNodeIndex].height + 1.0f;
            skeleton_info[currNodeIndex].children.push_back(adjacentNodeIndex);
            maxHeight = std::max(maxHeight, skeleton_info[adjacentNodeIndex].height);

            // sum it to help compute the tangent vector
            childrenMedian = childrenMedian + skeleton.skeleton_points.at(adjacentNodeIndex);
            childCnt += 1;
        }
        if (childCnt > 0) childrenMedian = childrenMedian / childCnt; else childrenMedian = skeleton.skeleton_points.at(currNodeIndex);
        Point3d tangentVector = childrenMedian - parentNode;
        tangentVector.normalize();
        //std::cout << "Degree of node " << currNodeIndex << " is " << edge_map[currNodeIndex].size();
        skeleton_info[currNodeIndex].is_branching = (edge_map.at(currNodeIndex).size() > 2);
        // skeleton_info[currNodeIndex].mIsEnd = (skeleton_info[currNodeIndex].mChildren.empty());
        skeleton_info[currNodeIndex].tangent_vector = tangentVector;
        currPointer += 1;
    }

    std::cout << "Start to compute terminal distance." << std::endl;
    // mark terminal distance
    for (int node_id = 0; node_id < skeleton.skeleton_points.size(); node_id++) {
        //if (skeleton_info[node_id].is_end) {
        if (edge_map[node_id].size() == 1) {
            if (skeleton_info[node_id].is_root) continue;
            skeleton_info[node_id].terminal_distance = 0;
            int parent = skeleton_info[node_id].parent_node_index;
            int terminal_distance = 1;
            //while (edge_map[parent].size() == 2) {
            while ( edge_map[parent].size() == 2) {
                if (skeleton_info[parent].is_root) break;
                skeleton_info[parent].terminal_distance = (terminal_distance ++);
                parent = skeleton_info[parent].parent_node_index;
            }
        }
    }
    stat_furthest_distance(rootNodeIndex, skeleton, skeleton_info);

    return rootNodeIndex;
}

void allocate_radius_for_branching_child(const Skeleton& user_skel, const unordered_map<int, SkeletonNodeInfo>& skeleton_info, unordered_map<int, double>& radius_info, int curr_node_id, double parent_radius) {
    int parent_node_id = skeleton_info.at(curr_node_id).parent_node_index;
    double parent_mass_supporting = skeleton_info.at(parent_node_id).mass_supporting;
    double parent_furthest_distance = skeleton_info.at(parent_node_id).furthest_terminal_distance;
    double curr_mass_supporting = skeleton_info.at(curr_node_id).mass_supporting;
    double curr_furthest_distance = skeleton_info.at(curr_node_id).furthest_terminal_distance;
    double theta = 1.7;
    //double radius = parent_radius;
    double radius = pow(pow(parent_radius, theta) * curr_furthest_distance / parent_furthest_distance, 1.0 / theta);
    if (skeleton_info.at(parent_node_id).children.size() == 1) {
        radius = parent_radius;
    }
    //double radius = parent_radius * curr_mass_supporting / parent_mass_supporting;
    radius_info[curr_node_id] = radius;
    for (int child_node_id : skeleton_info.at(curr_node_id).children) {
        allocate_radius_for_branching_child(user_skel, skeleton_info, radius_info, child_node_id, radius);
    }
}

// Add the constraint that the radius of child branch cannot be larger than 
// its parent's radius.
void apply_parent_child_constrains(int root_node_id, double aver_parent_radius, const unordered_map<int, SkeletonNodeInfo> & skeleton_info, unordered_map<int, double>& radius_info) {
    double parent_radius_sum = 0.0;
    int parent_branch_node_cnt = 0;
    int curr_node_id = root_node_id;
    vector<int> curr_branch_node_indices;
    while (skeleton_info.at(curr_node_id).children.size() == 1) {
        curr_branch_node_indices.push_back(curr_node_id);
        //double curr_radius = radius_info[curr_node_id];
        // apply constraint
        if (aver_parent_radius > 0 && skeleton_info.at(curr_node_id).furthest_terminal_distance < taper_threshold) {
            //std::cout << "Impose constraint!" << std::endl;
            radius_info[curr_node_id] = std::min(aver_parent_radius, radius_info[curr_node_id]);
        }         
        parent_radius_sum += radius_info[curr_node_id];
        parent_branch_node_cnt += 1;
        curr_node_id = skeleton_info.at(curr_node_id).children.front();
    }
    curr_branch_node_indices.push_back(curr_node_id);
    if (aver_parent_radius > 0) {
        radius_info[curr_node_id] = std::min(aver_parent_radius, radius_info[curr_node_id]);
    }
    parent_radius_sum += radius_info[curr_node_id];
    parent_branch_node_cnt += 1;

    double aver_curr_branch_radius = parent_radius_sum / parent_branch_node_cnt;
    // Constraint max radius on current branch
    int branch_node_cnt = 0;
    double prev_radius = radius_info[curr_branch_node_indices.front()];
    for (int node_id : curr_branch_node_indices) {
        radius_info[node_id] = std::min(radius_info[node_id], aver_curr_branch_radius * 4.0);
        if (branch_node_cnt > 2) {
            radius_info[node_id] = std::min(prev_radius, radius_info[node_id]);
        }
        prev_radius = radius_info[node_id]; 
        branch_node_cnt += 1;
    }

    for (int child_node_id : skeleton_info.at(curr_node_id).children) {
        //std::cout << "aver radius: " << aver_curr_branch_radius << std::endl;
        apply_parent_child_constrains(child_node_id, aver_curr_branch_radius, skeleton_info, radius_info);
    }
}

void calculate_radius_info(const Skeleton& user_skel, const std::vector<Point3d>& point_cloud, std::unordered_map<int, double>& radius_info) {
    radius_info.clear();
    zjl::OctreeXYZ<Point3d> octree(point_cloud);
    octree.build_index();

    unordered_map<int, SkeletonNodeInfo> skeleton_info;
    int analysis_res = analyze_skeleton(user_skel, skeleton_info);
    if (analysis_res == -1) {
        return;
    }

    // Compute trunk radius
    int next_id = analysis_res;
    double trunk_radius = 0.0;
    int trunk_node_count = 0;
    // while (true) {
    //     std::vector<int> neighbors;
    //     Point3d node = user_skel.skeleton_points.at(next_id);
    //     octree.search_k_neighbors(100, node, neighbors);
    //     double distance_sum = 0.0;
    //     for (int neighbor_index : neighbors) {
    //         distance_sum += (node - point_cloud[neighbor_index]).norm();
    //     }
    //     double radius = (distance_sum / neighbors.size());
    //     radius_info[next_id] = radius;
    //     trunk_radius += radius;

    //     trunk_node_count += 1;

    //     // decide next trunk node id
    //     if (skeleton_info[next_id].children.size() == 1) {
    //         next_id = skeleton_info[next_id].children[0];
    //     } else {
    //         break;
    //     }
    // }
    // trunk_radius = trunk_radius / trunk_node_count;

    // // allocate radius for rest skeleton node
    // for (int child_node_id : skeleton_info[next_id].children) {
    //     allocate_radius_for_branching_child(user_skel, skeleton_info, radius_info, child_node_id, trunk_radius);
    // }

    // ################
    // Compute radius based on nearby points
    int neighbor_num = 65; // for mve
    //int neighbor_num = 25; // for raw
    for (int skel_node_id = 0; skel_node_id < user_skel.skeleton_points.size(); skel_node_id ++) {
        std::vector<int> neighbors;
        Point3d node = user_skel.skeleton_points.at(skel_node_id);
        auto tangent_vector = skeleton_info[skel_node_id].tangent_vector;
        octree.search_k_neighbors(neighbor_num, node, neighbors);
        double distance_sum = 0.0;
        for (int neighbor_index : neighbors) {
            auto node_to_point = node - point_cloud[neighbor_index];
            auto angle = acos(node_to_point.dot(tangent_vector) / (node_to_point.norm() * tangent_vector.norm()));
            auto projection_distance = node_to_point.norm() * fabs(sin(angle));
            //distance_sum += (node - point_cloud[neighbor_index]).norm();
            distance_sum += projection_distance;
        }
        double radius = (distance_sum / neighbors.size());
        radius *= 1.1;
        //radius *= 1.0;
        radius_info[skel_node_id] = radius;

        // double radius = radius_info[skel_node_id];

        double branch_end_taper_threshold = 0.2;
        if (skeleton_info[skel_node_id].furthest_terminal_distance >= 0) {
            double terminal_distance = skeleton_info[skel_node_id].furthest_terminal_distance;
            radius = radius - radius * pow(math_e, -terminal_distance * 1.0 / branch_end_taper_threshold) * 0.99;
            radius_info[skel_node_id] = radius;
        }
    } 

    apply_parent_child_constrains(analysis_res, -1.0, skeleton_info, radius_info);

    // for (int node_id = 0; node_id < user_skel.skeleton_points.size(); node_id++) {
    //     if (skeleton_info[node_id].is_branching) {
    //         double radius_i = 0.0f;
    //         for (int child_id : skeleton_info[node_id].children) {
    //             radius_i += pow(radius_info[child_id], 1.5);
    //         }
    //         radius_i = pow(radius_i, 0.667);
    //         radius_info[node_id] = radius_i;
    //     }
    // }

    // ######################
    // Gaussian smooth
    // zjl::OctreeXYZ<Point3d> skel_octree(user_skel.skeleton_points);
    // skel_octree.build_index();
    // std::vector<int> skeleton_point_neighbor_indices;
    // int neighbor_num = 6;
    // unordered_map<int, double> radius_info_smoothed;
    // for (int node_id = 0; node_id < user_skel.skeleton_points.size(); node_id ++) {
    //     auto& skeleton_point = user_skel.skeleton_points[node_id];
    //     skel_octree.search_k_neighbors(neighbor_num, skeleton_point, skeleton_point_neighbor_indices);
    //     double radius_i = 0.0;
    //     double gaussian_weight_sum = 0.0;
    //     for (int i = 0; i < skeleton_point_neighbor_indices.size(); i ++) {
    //         int neighbor_index = skeleton_point_neighbor_indices[i];
    //         double neighbor_radius = radius_info[neighbor_index];
    //         double gaussian_weight = pow(math_e, -i * i * 1.0 / (neighbor_num * neighbor_num));
    //         radius_i += (neighbor_radius * gaussian_weight);
    //         gaussian_weight_sum += gaussian_weight;
    //     }
    //     radius_i /= (gaussian_weight_sum);
    //     radius_info_smoothed[node_id] = radius_i;
    // }
    // radius_info = radius_info_smoothed;

}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << argv[0] << "  xxx.obj(your skeleton) xxx.npts(raw pc) xxx.ri(output radius info)" << std::endl;
        return -1;
    }
    std::vector<Point3d> point_cloud;
    //Skeleton gt_skel;
    Skeleton your_skel;
    unordered_map<int, double> radius_info;

    zjl::IO::read_point_cloud_from_npts_without_normals(argv[2], point_cloud);
    //zjl::IO::read_graph_from_obj(argv[1], gt_skel.skeleton_points, gt_skel.skeleton_edges);
    zjl::IO::read_graph_from_obj(argv[1], your_skel.skeleton_points, your_skel.skeleton_edges);
    calculate_radius_info(your_skel, point_cloud, radius_info);
    write_radius_info_to_ri(argv[3], radius_info);
    return 0;
}
