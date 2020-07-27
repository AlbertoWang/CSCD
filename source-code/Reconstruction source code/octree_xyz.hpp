#pragma once
#include <iostream>
#include <cmath>
#include <string>
#include <cstring>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <random>
#include <functional>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

// for definitions
namespace zjl {
};

namespace zjl {
    using std::vector;
    using std::cout;
    using std::cerr;
    using std::endl;
    using std::sort;
    using std::priority_queue;
    using std::unordered_map;
    using std::unordered_set;
    //using std::unordered_set;

template<typename P>
struct Sphere {
    P center;
    double radius;

    Sphere(const P& c, const double r): center(c), radius(r) {
    }
};

template<typename P>
struct Box {
    P center;
    double x_extent;
    double y_extent;
    double z_extent;

    Box(const P& center, double x, double y, double z) 
    : center(center), x_extent(x), y_extent(y), z_extent(z)
    {}

    bool contains(const P& point) const{  
        return 
               point.x >= center.x - x_extent && point.x <= center.x + x_extent
            && point.y >= center.y - y_extent && point.y <= center.y + y_extent
            && point.z >= center.z - z_extent && point.z <= center.z + z_extent   ;
    } 
};

template<typename P>
struct BoundingBox {
    P center;
    P extent;
};

template<typename P>
bool intersects(const Sphere<P>& sphere, const Box<P>& box) {
    const P& c1 = box.center;
    const P& c2 = sphere.center;
    P hemi_diagonal = {box.x_extent, box.y_extent, box.z_extent}; 
    P c1c2_sub_hemi = P(fabs(c2.x - c1.x), fabs(c2.y - c1.y), fabs(c2.z - c1.z)) - hemi_diagonal;
    P distance = {std::max(0.0, c1c2_sub_hemi.x), std::max(0.0, c1c2_sub_hemi.y), std::max(0.0, c1c2_sub_hemi.z)}; 
    return distance.dot(distance) <= sphere.radius * sphere.radius;
} 

template<typename P>
bool sphere_contains(const Sphere<P>& sphere, const Box<P>& box) {
    const P& c1 = box.center;
    const P& c2 = sphere.center;
    const P& c1c2_abs = {fabs(c2.x - c1.x), fabs(c2.y - c1.y), fabs(c2.z - c1.z)};    
    const P& hemi_diag = {box.x_extent, box.y_extent, box.z_extent};
    return (c1c2_abs + hemi_diag).norm() <= sphere.radius;
}

template<typename P>
BoundingBox<P> compute_bounding_box(const vector<P>& raw_points) {
    if (raw_points.empty()) {
        std::cerr << "Error: no points" << endl;
        exit(-1);
    }
    P extent(0, 0, 0);
    P center(0, 0, 0);
    const double LD = std::numeric_limits<double>::lowest();
    const double MD = std::numeric_limits<double>::max();
    P max_point(LD, LD, LD);
    P min_point(MD, MD, MD);
    for (const auto& kv : raw_points) {
        max_point.x = std::max(kv.x, max_point.x);
        max_point.y = std::max(kv.y, max_point.y);
        max_point.z = std::max(kv.z, max_point.z);
        // ==
        min_point.x = std::min(kv.x, min_point.x);
        min_point.y = std::min(kv.y, min_point.y);
        min_point.z = std::min(kv.z, min_point.z);
    }
    center = (max_point + min_point) * 0.5;
    extent = (max_point - min_point) * 0.5;
    return {center, extent};
}

//struct OctreeNode;
template<typename P>
struct OctreeNode: public Box<P> {
    long index;

    vector<int> indices;

    int depth;

    OctreeNode* children[8];

    OctreeNode(P center, double length, int depth):
        Box<P>(center, length, length, length),
        depth(depth) {
        memset(children, 0, sizeof(OctreeNode*) * 8);
    }

    bool is_leaf() const {return children[0] == nullptr;}

};

/**
 * ####################################################
 * Octree Implementation
 * ####################################################
 */
template <typename P>
class OctreeXYZ {
private:
    const vector<P>& points;

    OctreeNode<P>* root;

    unordered_map<long, OctreeNode<P>*> octree_indices;

    const int MAX_DEPTH = 10;

    const int MAX_POINTS_PER_NODE = 100;

private:
    void create_root_node() {
        auto bbox = compute_bounding_box(points);
        double max_extent = std::max({bbox.extent.x, bbox.extent.y, bbox.extent.z});
        this->root = new OctreeNode<P>(bbox.center, max_extent, 0); 
        this->root->index = 0;
        for (int i = 0; i < points.size(); i ++) this->root->indices.push_back(i);
    }

    void create_octree(int current_depth, OctreeNode<P>* current_node) {
        // if (current_node == nullptr) {cerr << "Error..........????" << endl;}
        if (current_node->indices.empty()) return;
        if (current_node->indices.size() <= MAX_POINTS_PER_NODE || current_depth >= MAX_DEPTH) {
            this->octree_indices[current_node->index] = current_node;
            return;
        }

        int cnt = 0;
        for (int i : {-1, 1}) {
            for (int j : {-1, 1}) {
                for (int k : {-1, 1}) {
                    long index = (long) ((i + 1) * 2 + (j + 1) * 1 + (k + 1) / 2);
                    index = current_node->index | (index << (3 * current_depth + 3));
                    const double length = current_node->x_extent;
                    P center = {
                        current_node->center.x + i * length / 2, 
                        current_node->center.y + j * length / 2, 
                        current_node->center.z + k * length / 2
                    };
                    OctreeNode<P>* node = new OctreeNode<P>(center, length / 2, current_depth + 1);
                    node->index = index;
                    node->indices.reserve(current_node->indices.size() / 8 + 10);
                    current_node->children[cnt ++] = node;
                }
            }
        }
        for (int index : current_node->indices) {
            const P& point = points[index];
            const P& center = current_node->center; 
            int xi = point.x < center.x ? 0 : 1;
            int yj = point.y < center.y ? 0 : 1;
            int zk = point.z < center.z ? 0 : 1;
            int child_index = xi * 4 + yj * 2 + zk * 1;
            current_node->children[child_index]->indices.push_back(index);
        }
        current_node->indices.clear();
        for (OctreeNode<P>* node : current_node->children) {
            create_octree(current_depth + 1, node);
        }
    }

    bool check_validity() const{
        int cnt = 0;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        for (const P& point : points) {
            if (point.x == nan || point.y == nan || point.z == nan
            || point.x == inf || point.y == inf || point.z == inf     ) {
                cerr << "Invalid input, at line: " << cnt << endl;
                return false;
            }
            cnt += 1;
        }
        return true;
    }

    void search_neighbors_in_nodes(const vector<long>& candidate_leaves, const P& point, std::priority_queue<int, vector<int>, std::function<bool(int, int)>>& queue) const {
        size_t capacity = 0;
        for (int leaf_index : candidate_leaves) capacity += this->octree_indices.at(leaf_index)->indices.size();
        queue = priority_queue<int, vector<int>, std::function<bool(int, int)>>([&](const int index1, const int index2){
            const P& p1 = points[index1];
            const P& p2 = points[index2];
            return (p1 - point).norm() > (p2 - point).norm();
        });
        for (long leaf_index : candidate_leaves) {
            for (int point_index : octree_indices.at(leaf_index)->indices) {
                queue.push(point_index);
            }
        }
    }

    long search_nearest_cell(const P& point) const {
        priority_queue<long, vector<long>, std::function<bool(long, long)>> cell_indices_heap([&](const long index1, const long index2) {
            const OctreeNode<P>* node1 = octree_indices.at(index1);
            const OctreeNode<P>* node2 = octree_indices.at(index2);
            return (point - node1->center).norm() > (point - node2->center).norm(); 
        });
        for (const auto& kv : octree_indices) {
            if (kv.second == nullptr) continue;
            if (!kv.second->is_leaf()) continue;
            if (kv.second->contains(point)) return kv.first;
            cell_indices_heap.push(kv.first);
        }
        return cell_indices_heap.top();
    }

    void determine_candidates_within_distance(double radius, const P& point, vector<long>& candidates) const{
        Sphere<P> sphere(point, radius);
        candidates.clear();
        //===========
        vector<OctreeNode<P>*> visiting_queue;
        if (intersects(sphere, *root)) visiting_queue.push_back(root);
        size_t current_visit = 0;
        for (; current_visit < visiting_queue.size(); current_visit ++) {
            const OctreeNode<P>* visiting = visiting_queue[current_visit];
            //if (visiting == nullptr) cerr << "Error: null pointer" << endl;
            if (visiting->is_leaf()) {
                if (octree_indices.find(visiting->index) == octree_indices.end()) continue;
                if (octree_indices.at(visiting->index) == nullptr) continue;
                candidates.push_back(visiting->index);
            } else {
                for (OctreeNode<P>* child : visiting->children) {
                    if (intersects(sphere, *child)) {
                        visiting_queue.push_back(child);
                    }
                }
            }
        }
        
    }

public:
    OctreeXYZ(const vector<P>& points): points(points), root(nullptr) {
    }

    OctreeXYZ(const OctreeXYZ<P>& another) = delete;
    OctreeXYZ(OctreeXYZ<P>&& another) = delete;
    OctreeXYZ& operator= (const OctreeXYZ<P>& another) = delete;
    OctreeXYZ& operator= (OctreeXYZ<P>&& another) = delete;

    ~OctreeXYZ() {
        for (auto& kv : octree_indices) {
            if (kv.second == nullptr) continue;
            delete(kv.second);
        }
        octree_indices.clear();
    }


    void build_index() {
        if (!check_validity()) {
            cerr << "Error: Fail to pass the validity check, exit." << endl;
            exit(-1);
        }
        if (points.empty()) {
            cerr << "Warning: input is an empty list" << endl;
            return;
        }
        // cerr << "Debug: clearing octree indices... " << endl;
        for (auto& kv : octree_indices) {
            if (kv.second == nullptr) continue;
            delete(kv.second);
        }
        this->octree_indices.clear();

        create_root_node();
        create_octree(0, root);
    }


    void search_k_neighbors(int k, const P& point, std::vector<int>& neighbors) const {
        if (k >= points.size() || k <= 0) {
            cerr << "Error: invalid k: " << k << endl;
            return;
        }
        neighbors.clear();

        const auto& big_first_comp = [&] (const int index1, const int index2) {
            return (points[index1] - point).norm() <= (points[index2] - point).norm();
        };
        long leaf_node_index = search_nearest_cell(point);
        // cerr << "Debug: " << "nearest cell center: " << octree_indices[leaf_node_index]->center.x << ", "
        //<< octree_indices[leaf_node_index]->center.y << ", " << octree_indices[leaf_node_index]->center.z << endl;
        priority_queue<int, vector<int>, std::function<bool(int, int)>> queue(big_first_comp);

        unordered_set<int> point_indices_set;
        unordered_set<long> cell_indices_set;
        double leaf_size = octree_indices.at(leaf_node_index)->x_extent;

        Sphere<P> search_range(point, leaf_size);
        while (queue.size() < k) {
            vector<long> candidates;
            determine_candidates_within_distance(search_range.radius, search_range.center, candidates);
            for (long candidate_index : candidates) {
                if (cell_indices_set.find(candidate_index) != cell_indices_set.end()) continue;
                if (sphere_contains(search_range, *octree_indices.at(candidate_index)))
                    cell_indices_set.insert(candidate_index);
                for (int point_index : octree_indices.at(candidate_index)->indices) {
                    if (point_indices_set.find(point_index) != point_indices_set.end()) continue;
                    if ((points[point_index] - point).norm() <= search_range.radius) {
                        queue.push(point_index);
                        point_indices_set.insert(point_index);
                    }
                    if (queue.size() > k) queue.pop();
                }

            }
            search_range.radius += leaf_size;
        }

        while (!queue.empty()) {
            neighbors.push_back(queue.top());
            queue.pop();
        }
        std::reverse(neighbors.begin(), neighbors.end()); 
    }

    void search_neighbors_within_distance(const P& point, double distance, std::vector<int>& neighbor_indices ) const {
        if (distance <= 0.0) {
            cerr << "Error: invalid radius: " << distance << endl;
            return;
        }
        neighbor_indices.clear();
        vector<long> candidate_leaves;
        determine_candidates_within_distance(distance, point, candidate_leaves);

        //cerr << "Debug: octree_indices.size(): " << octree_indices.size() << endl;

        priority_queue<int, vector<int>, std::function<bool(int, int)>> queue;
        search_neighbors_in_nodes(candidate_leaves, point, queue);

        while (!queue.empty()) {
            int next_index = queue.top();
            queue.pop();
            const P& neighboring_point = points[next_index];
            //cerr << "Debug: " << neighboring_point.x << " " << neighboring_point.y << " " << neighboring_point.z << ", distance: " << (neighboring_point - point).norm() << endl;
            if ((point - neighboring_point).norm() >= distance) break;
            
            neighbor_indices.push_back(next_index);
        }
    }

    const unordered_map<long, zjl::OctreeNode<P>* >& get_nodes() const {return this->octree_indices;}

};
};