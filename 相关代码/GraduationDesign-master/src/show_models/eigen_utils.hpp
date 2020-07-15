#pragma once
#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace zjl {
namespace Util {
//public:
    template<typename P>
     void xyz_points_to_eigen_matrix(const std::vector<P>&point_cloud, Eigen::MatrixXf& point_cloud_matrix) {
        using namespace std;
        point_cloud_matrix.resize(point_cloud.size(), 3);
        for (size_t row = 0; row < point_cloud.size(); row ++) {
            point_cloud_matrix(row, 0) = point_cloud[row].x;
            point_cloud_matrix(row, 1) = point_cloud[row].y;
            point_cloud_matrix(row, 2) = point_cloud[row].z;
        }
        cout << "Succeed in converting point cloud to eigen matrix" << endl;
    }

    template<typename E>
    void ij_edge_to_eigen_matrix(const std::vector<E>&edges, Eigen::MatrixXi& edge_matrix) {
        using namespace std;
        edge_matrix.resize(edges.size(), 2);
        for (size_t row = 0; row < edges.size(); row ++) {
            edge_matrix(row, 0) = edges[row].vi;
            edge_matrix(row, 1) = edges[row].vj;
        }
        cout << "Succeed in converting edges to eigen matrix" << endl;
    }

    template<typename T>
     void ijk_faces_to_eigen_matrix(const std::vector<T>&faces, Eigen::MatrixXi& face_matrix) {
        using namespace std;
        face_matrix.resize(faces.size(), 3);
        for (size_t row = 0; row < faces.size(); row ++) {
            face_matrix(row, 0) = faces[row].vi;
            face_matrix(row, 1) = faces[row].vj;
            face_matrix(row, 2) = faces[row].vk;
        }
        cout << "Succeed in converting faces to eigen matrix" << endl;
    }

};

};
