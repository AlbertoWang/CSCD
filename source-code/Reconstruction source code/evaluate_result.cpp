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

using namespace std;

struct Skeleton {
    std::vector<Point3d> points;
    std::vector<Edge> edges;
};

void analyze_skeleton(const Skeleton& skel1, std::vector<Point3d*>& skel1_valid_points,  std::unordered_map<const Point3d*, Vector3d>& s1_tangents) {
    skel1_valid_points.clear();
    s1_tangents.clear();

    // convert skeleton into graph representation
    std::unordered_map<int, std::vector<int> > skel_edges;
    std::unordered_map<int, std::unordered_set<int> > skel_edges_s;
    for (const auto& edge : skel1.edges) {
        skel_edges_s[edge.vi].insert(edge.vj);
        skel_edges_s[edge.vj].insert(edge.vi);
    }
    for (auto& kv : skel_edges_s) {
        for (auto v_i : kv.second) {
            skel_edges[kv.first].push_back(v_i);
        }
    }

    // Filter invalid points and compute tangents
    for (auto& kv : skel_edges) {
        int vert_index = kv.first;
        auto& adj_vert_indices = kv.second;
        if (kv.second.size() != 2) continue;
        skel1_valid_points.push_back(new Point3d(skel1.points.at(vert_index)));
        auto prev_point = skel1.points.at(kv.second.front());
        auto next_point = skel1.points.at(kv.second.back());

        Vector3d tangent = (next_point - prev_point);
        tangent.normalize();
        s1_tangents.insert({{(skel1_valid_points.back()), tangent}});
        
        //s1_tangents[&skel1_valid_points.back()] = tangent;
    }
}

void compute_nearest_point_pair_2(const std::vector<Point3d*>& skel1_valid_points, const std::vector<Point3d*>& skel2_valid_points, unordered_map<const Point3d*, const Point3d*>& s1_to_s2, unordered_map<const Point3d*, const Point3d*>& s2_to_s1) {
    s1_to_s2.clear();
    s2_to_s1.clear();
    for (const auto& point : skel1_valid_points) {
        double nearest_distance = std::numeric_limits<double>::max();
        const Point3d* nearest_point_ptr = nullptr;
        for (const auto& point2 : skel2_valid_points) {
            double distance = (*point - *point2).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_point_ptr = point2;
            } 
        }
        s1_to_s2[point] = nearest_point_ptr;
    }
    for (const auto& point : skel2_valid_points) {
        double nearest_distance = std::numeric_limits<double>::max();
        const Point3d* nearest_point_ptr = nullptr;
        for (const auto& point1 : skel1_valid_points) {
            double distance = (*point - *point1).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_point_ptr = point1;
            } 
        }
        s2_to_s1[point] = nearest_point_ptr;
    }
}

double compute_tangent_distance(const Skeleton& skel, const Skeleton& gt) {
    
    double distance = 0.0;
    std::vector<Point3d*> skel1_valid_points;    
    std::vector<Point3d*> skel2_valid_points;    
    unordered_map<const Point3d*, Vector3d> s1_tangents;
    unordered_map<const Point3d*, Vector3d> s2_tangents;
    analyze_skeleton(skel, skel1_valid_points, s1_tangents);
    analyze_skeleton(gt, skel2_valid_points, s2_tangents);
    unordered_map<const Point3d*, const Point3d*> s1_to_s2;
    unordered_map<const Point3d*, const Point3d*> s2_to_s1;
    compute_nearest_point_pair_2(skel1_valid_points, skel2_valid_points, s1_to_s2, s2_to_s1);
    //cout << "s1 tangent size: " << s1_tangents.size() << " ,s2 tangent size: " << s2_tangents.size() << endl;
    int cnt = 0;
    for (const auto& p : skel1_valid_points) {
        const Point3d* point1 = p;
        const Point3d* point2 = s1_to_s2[point1];
        const auto& tangent_1 = s1_tangents.at(point1);
        const auto& tangent_2 = s2_tangents.at(point2);
        if (tangent_1.norm() < 1e-7 || tangent_2.norm() < 1e-7) continue;
        double angle = acos(abs(tangent_1.dot(tangent_2)) / (tangent_1.norm() * tangent_2.norm()));
        if (angle != angle) angle = 0.0;
        distance += angle;
        cnt += 1;
    }
    for (const auto& p : skel2_valid_points) {
        const Point3d* point2 = p;
        const Point3d* point1 = s2_to_s1[point2];
        const auto& tangent_1 = s1_tangents.at(point1);
        const auto& tangent_2 = s2_tangents.at(point2);
        if (tangent_1.norm() < 1e-7 || tangent_2.norm() < 1e-7) continue;
        double angle = acos(abs(tangent_1.dot(tangent_2)) / (tangent_1.norm() * tangent_2.norm()));
        if (angle != angle) angle = 0.0;
        distance += angle;
        cnt += 1;
    }
        //cout << "distance: " << distance << " cnt: " << cnt << std::endl;
    distance /= cnt;

    for (auto ptr : skel1_valid_points) delete(ptr);
    for (auto ptr : skel2_valid_points) delete(ptr);
    return distance;
}

void compute_nearest_point_pairs(const Skeleton& skel1, const Skeleton& skel2, unordered_map<const Point3d*, pair<const Point3d*, double> >& s1_to_s2, unordered_map<const Point3d*, pair<const Point3d*, double> >& s2_to_s1) {
    s1_to_s2.clear();
    s2_to_s1.clear();
    for (const auto& point : skel1.points) {
        double nearest_distance = std::numeric_limits<double>::max();
        const Point3d* nearest_point_ptr = nullptr;
        for (const auto& point2 : skel2.points) {
            double distance = (point - point2).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_point_ptr = & point2;
            } 
        }
        s1_to_s2[&point] = {nearest_point_ptr, nearest_distance};
    }
    for (const auto& point : skel2.points) {
        double nearest_distance = std::numeric_limits<double>::max();
        const Point3d* nearest_point_ptr = nullptr;
        for (const auto& point1 : skel1.points) {
            double distance = (point - point1).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_point_ptr = & point1;
            } 
        }
        s2_to_s1[&point] = {nearest_point_ptr, nearest_distance};
    }
}

double compute_hausdorff_distance(const Skeleton& skel, const Skeleton& gt) {
    double distance = 0.0;
    unordered_map<const Point3d*, pair<const Point3d*, double> > s1_to_s2; unordered_map<const Point3d*, pair<const Point3d*, double> > s2_to_s1;
    compute_nearest_point_pairs(skel, gt, s1_to_s2, s2_to_s1); 
    for (auto& pp : s1_to_s2) {
        distance = std::max(distance, pp.second.second);
    }
    for (auto& pp : s2_to_s1) {
        distance = std::max(distance, pp.second.second);
    }
    return distance;
}

double compute_chamfer_distance(const Skeleton& skel, const Skeleton& gt) {
    double distance = 0.0;
    unordered_map<const Point3d*, pair<const Point3d*, double> > s1_to_s2; unordered_map<const Point3d*, pair<const Point3d*, double> > s2_to_s1;
    compute_nearest_point_pairs(skel, gt, s1_to_s2, s2_to_s1); 
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
    //distance /= (skel.points.size() + gt.points.size());
    return distance;

}


double compute_mean_distance(const Skeleton& skel, const Skeleton& gt) {
    double distance = 0.0;
    unordered_map<const Point3d*, pair<const Point3d*, double> > s1_to_s2; unordered_map<const Point3d*, pair<const Point3d*, double> > s2_to_s1;
    compute_nearest_point_pairs(skel, gt, s1_to_s2, s2_to_s1); 
    for (auto& pp : s1_to_s2) {
        distance += pp.second.second;
    }
    for (auto& pp : s2_to_s1) {
        distance += pp.second.second;
    }
    distance /= (skel.points.size() + gt.points.size());
    return distance;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " xxx.obj(output by algorithm) xxx.obj(ground truth)" << endl;
        return -1;
    }
    // cout << "Hello world" << endl;
    Skeleton result;
    Skeleton gt;
    zjl::IO::read_graph_from_obj(argv[1], result.points, result.edges);
    zjl::IO::read_graph_from_obj(argv[2], gt.points, gt.edges);

    double hausdorff_dis = compute_hausdorff_distance(result, gt);
    double mean_dis = compute_mean_distance(result, gt);
    double chamfer_dis = compute_chamfer_distance(result, gt);
    double tangent_dis = compute_tangent_distance(result, gt);
    cout 
        << argv[1]          << '\t' 
        << hausdorff_dis    << '\t' 
        << mean_dis      << '\t' 
        << tangent_dis      << std::endl;
    FILE *fp = fopen("./evaluation.log", "a+");
    if (fp == nullptr) {
        cout << "Fail to open file" << endl;
        return 0;
    }
    fprintf(fp, "%s\t%.4f\t%.4f\t%.4f\n", argv[1], hausdorff_dis, mean_dis, tangent_dis);
    return 0;
}