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
#include "string_utils.hpp"

namespace zjl {
class IO {
public:

    template<typename P>
    static void read_point_cloud_from_npts_without_normals(const std::string& filepath, std::vector<P>&point_cloud) {
        using namespace std;
        ifstream ifs(filepath);
        if (!ifs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        point_cloud.clear();
        string line;
        string keyword;
        double x = 0, y = 0, z = 0;
        while (std::getline(ifs, line)) {
            if (line.find("nan") != line.npos) continue;
            istringstream iss(line);
            if (!(iss >> x >> y >> z)) {
                cerr << "Error: failed to read the keyword from line: " << line << endl;
                continue;
            }
            point_cloud.push_back({x, y, z});
        }
        cout << "Read " << point_cloud.size() << " points" << endl;
    }

    /* ROSA skeleton reader */
    template<typename P, typename E>
    static void read_graph_from_rosa_cg(const std::string& filepath, std::vector<P>&points, std::vector<E>& edges) {
        using namespace std;
        ifstream ifs(filepath);
        if (!ifs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        points.clear();
        edges.clear();
        string line;
        int vertex_cnt = 0;
        while (std::getline(ifs, line)) {
            auto parts = zjl::Util::string_split_by_ws(line);
            if (parts.empty()) continue;
            if (parts.front() == "#") continue;
            if (parts.front() == "v") {
                // std::cerr << "Debug: Find CN --- " << std::endl;
                if (parts.size() < 4) {
                    std::cerr << "less than three floats, the line: " << line << std::endl;
                    continue;
                }
                float x = atof(parts[1].data()); 
                float y = atof(parts[2].data()); 
                float z = atof(parts[3].data()); 
                points.push_back({x, y, z});
            } else if (parts.front() == "e") {
                if (parts.size() < 3) {
                    std::cerr << "less than two integers, the line: " << line << std::endl;
                    continue;
                }
                int vi = atof(parts[1].data()); 
                int vj = atof(parts[2].data()); 
                edges.push_back({vi - 1, vj - 1});
            }
        }
        cout << "Read " << points.size() << " points, " << edges.size() << " edges." << endl;
    }

    /* L1-median skeleton reader */
    template<typename P, typename E>
    static void read_graph_from_skel(const std::string& filepath, std::vector<P>&points, std::vector<E>& edges) {
        using namespace std;
        ifstream ifs(filepath);
        if (!ifs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        points.clear();
        edges.clear();
        string line;
        int vertex_cnt = 0;
        while (std::getline(ifs, line)) {
            auto parts = zjl::Util::string_split_by(line, " ");
            if (parts.empty()) continue;
            if (parts.front() == "CN") {
                // std::cerr << "Debug: Find CN --- " << std::endl;
                int bn = atoi(parts[1].data()); 
                if (bn < 0) {
                    std::cerr << "Invalid bn : " << bn << endl;
                }
                for (int i = 0; i < bn; i ++) {
                    std::getline(ifs, line);
                    auto branch_parts = zjl::Util::string_split_by(line, " ");
                    // if (branch_parts.empty()) continue;
                    int vn = atoi(branch_parts[1].data()); 
                    for (int j = 0; j < vn; j ++) {
                        std::getline(ifs, line); 
                        auto vline_parts = zjl::Util::string_split_by_ws(line);
                        if (vline_parts.empty()) continue;
                        if (vline_parts.size() < 3) {
                            std::cerr << "Error: fewer than three floats, line: " << line << std::endl;
                        }
                        float x = atof(vline_parts[0].data());
                        float y = atof(vline_parts[1].data());
                        float z = atof(vline_parts[2].data());
                        points.push_back({x, y, z});
                    }
                    for (int j = 0; j < vn - 1; j ++) {
                        // ???
                        int vi = vertex_cnt + j + 0;
                        int vj = vertex_cnt + j + 1;
                        edges.push_back({vi, vj});
                    }
                    vertex_cnt += vn;
                }
            }
        }
        cout << "Read " << points.size() << " points, " << edges.size() << " edges." << endl;
    }

    template<typename P, typename E>
    static void read_graph_from_obj(const std::string& filepath, std::vector<P>&points, std::vector<E>& edges) {
        using namespace std;
        ifstream ifs(filepath);
        if (!ifs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        points.clear();
        edges.clear();
        string line;
        string keyword;
        double x = 0, y = 0, z = 0;
        int vi = 0, vj = 0;
        int edge_cnt = 0;
        while (std::getline(ifs, line)) {
            if (line.find("nan") != line.npos) continue;
            istringstream iss(line);
            if (!(iss >> keyword)) {
                cerr << "Error: failed to read the keyword from line: " << line << endl;
                continue;
            }
            if (keyword == "#" || keyword == "o" || keyword == "mtllib") continue;
            if (keyword == "v") {
                if (!(iss >> x >> y >> z)) {
                    cerr << "Error: failed to read six float number from line: " << line << endl;
                    break;
                }
                points.push_back({x, y, z});
            }
            if (keyword == "l") {
                if (!(iss >> vi >> vj)) {
                    cerr << "Error: failed to read two int number from line: " << line << endl;
                    break;
            }
            edges.push_back({vi - 1, vj - 1});
            // skeleton_graph[vi - 1].insert(vj - 1);
            // skeleton_graph[vj - 1].insert(vi - 1);
            edge_cnt += 1;
            }
        }
        cout << "Read " << points.size() << " points, " << edge_cnt << " edges." << endl;
    }

    template<typename P>
    static void read_point_cloud_from_obj_without_normals(const std::string& filepath, std::vector<P>& point_cloud) {
        using namespace std;
        ifstream ifs(filepath);
        if (!ifs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        point_cloud.clear();
        string line;
        string keyword;
        double x = 0, y = 0, z = 0;
        int point_num = 0;
        while (std::getline(ifs, line)) {
            if (line[0] != 'v') continue;
            istringstream iss(line);
            iss >> keyword >> x >> y >> z;
            point_cloud.push_back({x, y, z});
        }
        cout << "Read " << point_cloud.size() << " points from" << filepath  << endl;
    }

    template<typename P>
    static void read_point_cloud_from_ply_without_normals(const std::string& filepath, std::vector<P>& point_cloud) {
        using namespace std;
        ifstream ifs(filepath);
        if (!ifs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        point_cloud.clear();
        string line;
        string keyword;
        double x = 0, y = 0, z = 0;
        int point_num = 0;
        while (std::getline(ifs, line)) {
            istringstream iss(line);
            if (!(iss >> keyword)) {
                cerr << "Error: failed to read the keyword from line: " << line << endl;
                continue;
            }
            if (keyword == "end_header" || keyword == "end_header\n") {
                cout << "Debug: Successfully get the end_header" << endl;
                break;
            } else if (keyword == "format") {
                iss >> keyword; 
                if (keyword != "ascii") {
                    cout << "Warning: not an ascii file" << endl;
                }
            } else if (keyword == "element") {
                iss >> keyword; 
                if (keyword == "vertex") {
                    iss >> point_num; 
                    cout << "Debug: read the number of vertices: " << point_num << endl;
                } else if (keyword == "face") {
                    cout << "Warn: Do not expect face element here!! " << endl;
                }
            }
        }
        int read_cnt = 0;
        while (std::getline(ifs, line)) {
            if (line.find("nan") != line.npos) continue;
            istringstream iss(line);
            if (read_cnt < point_num) {
                if (!(iss >> x >> y >> z)) {
                    cerr << "Error: failed to read six float number from line: " << line << endl;
                    break;
                }
                point_cloud.push_back({x, y, z});
            }             
            read_cnt += 1;
        }
        cout << "Read " << point_cloud.size() << " points from" << filepath  << endl;
    }

    template<typename P, typename T>
    static void read_trimesh_from_ply(const std::string& mesh_file_path, std::vector<P>& vertices, std::vector<T>& triangles) {
        using namespace std;
        ifstream ifs(mesh_file_path);
        if (!ifs.is_open()) {
            cerr << "Error: can't open " << mesh_file_path << endl;
            return;
        }
        vertices.clear();
        triangles.clear();
        string line;
        string keyword;
        double x = 0, y = 0, z = 0;
        int vi = 0, vj = 0, vk = 0;
        int point_num = 0;
        int face_num = 0;
        while (std::getline(ifs, line)) {
            istringstream iss(line);
            if (!(iss >> keyword)) {
                cerr << "Error: failed to read the keyword from line: " << line << endl;
                continue;
            }
            if (keyword == "end_header" || keyword == "end_header\n") {
                cout << "Debug: Successfully get the end_header" << endl;
                break;
            } else if (keyword == "format") {
                iss >> keyword; 
                if (keyword != "ascii") {
                    cout << "Warning: not an ascii file" << endl;
                }
            } else if (keyword == "element") {
                iss >> keyword; 
                if (keyword == "vertex") {
                    iss >> point_num; 
                    cout << "Debug: read the number of vertices: " << point_num << endl;
                } else if (keyword == "face") {
                    iss >> face_num;
                    cout << "Debug: read the number of faces: " << face_num << endl;
                }
            }
        }
        int read_cnt = 0;
        while (std::getline(ifs, line)) {
            if (line.find("nan") != line.npos) continue;
            istringstream iss(line);
            if (read_cnt < point_num) {
                if (!(iss >> x >> y >> z)) {
                    cerr << "Error: failed to read six float number from line: " << line << endl;
                    break;
                }
                vertices.push_back({x, y, z});
            } else if (read_cnt < point_num + face_num) {
                if (!(iss >> vi >> vi >> vj >> vk)) {
                    cerr << "Error: failed to read four int number from line: " << line << endl;
                    break;
                }
                triangles.push_back({vi, vj, vk});
            }
            read_cnt += 1;
        }
        cout << "Read " << vertices.size() << " points, " << triangles.size() << " triangles" << endl;
    }

    template<typename P, typename E>
    static void write_graph_to_obj(const std::string& filepath, const std::vector<P>& points, const std::vector<E>& edges) {
        using namespace std;
        ofstream ofs(filepath); 
        if (! ofs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        for (const auto& point : points) {
            ofs << "v " << point.x << " " << point.y << " " << point.z << std::endl;
        }
        for (const auto& edge : edges) {
            ofs << "l " << edge.vi + 1 << " " << edge.vj + 1 << endl;
        }
        std::cout << "Write " << points.size() << " points, " << edges.size() << " edges to " << filepath << endl;
    }



    template<typename P, typename T>
    static void write_trimesh_to_off(const std::string& filepath, const std::vector<P>& vertices, const std::vector<T>& triangles) {
        using namespace std;
        ofstream ofs(filepath); 
        if (! ofs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        ofs << "OFF" << endl;
        ofs << vertices.size() << " " << triangles.size() << " 0";
        for (const auto& vertex : vertices) {
            ofs << endl << vertex.x << " " << vertex.y << " " << vertex.z;
        }
        for (const auto& triangle : triangles) {
            ofs << endl << triangle.vi << " " << triangle.vj << " " << triangle.vk;
        }
    }

    template<typename P, typename T>
    static void write_trimesh_to_ply_without_normals(const std::string& filepath, const std::vector<P>& vertices, const std::vector<T>& faces) {
        using namespace std;
        FILE* fp = fopen(filepath.data(), "wb");
        if (fp == nullptr) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        // Write header
        fprintf(fp, "ply\n");
        fprintf(fp, "format ascii 1.0\n");
        fprintf(fp, "comment generated by io_xyz.hpp\n");
        fprintf(fp, "element vertex %d\n", vertices.size());
        fprintf(fp, "property float x\n");
        fprintf(fp, "property float y\n");
        fprintf(fp, "property float z\n");
        fprintf(fp, "element face %d\n", faces.size());
        fprintf(fp, "property list uchar uint vertex_indices\n");
        fprintf(fp, "end_header\n");
        // write data
        for (size_t i = 0; i < vertices.size(); i ++) {
            fprintf(fp, "%.4f %.4f %.4f\n", vertices[i].x,  vertices[i].y, vertices[i].z);
        }
        
        for (size_t i = 0; i < faces.size(); i ++) {
            fprintf(fp, "%d %d %d %d\n", 3, faces[i].vi, faces[i].vj, faces[i].vk);
        }
        fclose(fp);
        std::cout << "Write " << vertices.size() << "vertices " << faces.size() << " faces to file" << filepath << std::endl;
    }

    template<typename P>
    static void write_point_cloud_to_ply_without_normals(const std::string& filepath, std::vector<P>&point_cloud) {
        using namespace std;
        FILE* fp = fopen(filepath.data(), "wb");
        if (fp == nullptr) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        // Write header
        fprintf(fp, "ply\n");
        fprintf(fp, "format ascii 1.0\n");
        fprintf(fp, "comment generated by io_xyz.hpp\n");
        fprintf(fp, "element vertex %d\n", point_cloud.size());
        fprintf(fp, "property float x\n");
        fprintf(fp, "property float y\n");
        fprintf(fp, "property float z\n");
        fprintf(fp, "end_header\n");
        // write data
        for (size_t i = 0; i < point_cloud.size(); i ++) {
            fprintf(fp, "%.4f %.4f %.4f\n", point_cloud[i].x,  point_cloud[i].y, point_cloud[i].z);
        }
        fclose(fp);
        std::cout << "Write " << point_cloud.size() << " points to file" << filepath << std::endl;
    }

    template<typename P>
    static void write_point_cloud_to_npts_without_normals(const std::string& filepath, const std::vector<P>&point_cloud) {
        using namespace std;
        if (point_cloud.empty()) {
            cerr << "Warning: point cloud is empty!" << endl;
        }
        ofstream ofs(filepath);
        if (!ofs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        // write points
        for (auto& point : point_cloud) {
            ofs << point.x << " " << point.y << " " << point.z << endl;
        }
        std::cout << "Write " << point_cloud.size() << " points to file" << filepath << std::endl;
    }

    template<typename P>
    static void write_point_cloud_to_off_without_normals(const std::string& filepath, const std::vector<P>&point_cloud) {
        using namespace std;
        if (point_cloud.empty()) {
            cerr << "Warning: point cloud is empty!" << endl;
        }
        ofstream ofs(filepath);
        if (!ofs.is_open()) {
            cerr << "Error: can't open " << filepath << endl;
            return;
        }
        ofs << "OFF" << std::endl << point_cloud.size() 
            << " " << 0 << std::endl;
        // write points
        for (auto& point : point_cloud) {
            ofs << point.x << " " << point.y << " " << point.z << endl;
        }
        std::cout << "Write " << point_cloud.size() << " points to file" << filepath << std::endl;
    }
};

};
