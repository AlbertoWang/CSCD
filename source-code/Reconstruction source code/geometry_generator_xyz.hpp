#include <vector>
#include <algorithm>
#include <iostream>

namespace zjl {
namespace Geometry {

    template<typename P, typename T>
    void generate_cylinder(int loop_count, int sample_count_per_loop, float radius, float height, std::vector<P>& vertices, std::vector<T>& faces) {
        vertices.clear();
        faces.clear();
        if (loop_count < 2 || sample_count_per_loop < 3 || radius < 0.0f || height < 0.0f) {
            std::cerr << "Error: Invalid argument, loop_count: " << loop_count << ", sample_count_per_loop:" << sample_count_per_loop
            << ", radius:" << radius << ", height:" << height;
            return;
        }
        if (sample_count_per_loop % 2 == 1) {
            sample_count_per_loop += 1;
        }
        const double PI = 3.1415926;
        int vertex_count = loop_count * sample_count_per_loop + 2;
        int face_count = loop_count * sample_count_per_loop * 2;

        // create vertices
        // the last two vertices are at the bottom side and top side, respectively.
        int vertex_index = 0, face_index = 0;
        for (int loop_index = 0; loop_index < loop_count; loop_index++) {
            const float y = 0.0f + loop_index * 1.0 / (loop_count - 1) * height - 0.5 * height;
            for (int sample_index = 0; sample_index < sample_count_per_loop; sample_index++) {
                const float radian = sample_index * 1.0 / sample_count_per_loop * 2.0 * PI;
                const float x = radius * std::cos(radian);
                const float z = radius * std::sin(radian);
                vertices.push_back({x, y, z});
            }
        }
        //cylinder->mVertexNormals.row(vertex_index) = RowVector3f(0, -1, 0); // test normal
        vertices.push_back({0.0f, -0.5 * height, 0.0f});

        //cylinder->mVertexNormals.row(vertex_index) = RowVector3f(0, 1, 0); // test normal
        vertices.push_back({0.0f, 0.5 * height, 0.0f});

        // create faces(triangles)
        for (int loop_index = 1; loop_index < loop_count; loop_index++) {
            //const float y = 0.0f + loop_index * 1.0 / loop_count * height;
            for (int sample_index = 0; sample_index < sample_count_per_loop; sample_index++) {
                int vert1 = 0, vert2 = 0, vert3 = 0, vert4 = 0, vert5 = 0, vert6 = 0;
                if (sample_index % 2 == 0) {
                    // clockwise
                    vert1 = loop_index * sample_count_per_loop + sample_index;
                    vert2 = (loop_index - 1) * sample_count_per_loop + sample_index;
                    vert3 = (loop_index - 1) * sample_count_per_loop + (sample_index - 1 + sample_count_per_loop) % sample_count_per_loop;
                    vert4 = loop_index * sample_count_per_loop + sample_index;
                    vert5 = (loop_index - 1) * sample_count_per_loop + (sample_index + 1) % sample_count_per_loop;
                    vert6 = (loop_index - 1) * sample_count_per_loop + sample_index;
                } else {
                    vert1 = loop_index * sample_count_per_loop + sample_index;
                    vert2 = (loop_index - 1) * sample_count_per_loop + sample_index;
                    vert3 = loop_index * sample_count_per_loop + (sample_index - 1 + sample_count_per_loop) % sample_count_per_loop;
                    vert4 = loop_index * sample_count_per_loop + sample_index;
                    vert5 = loop_index * sample_count_per_loop + (sample_index + 1) % sample_count_per_loop;
                    vert6 = (loop_index - 1) * sample_count_per_loop + sample_index;
                }
                faces.push_back({vert1, vert2, vert3});
                faces.push_back({vert4, vert5, vert6});
            }
        }
        // bottom side
        for (int sample_index = 0; sample_index < sample_count_per_loop; sample_index++) {
            int vert1 = sample_index;
            int vert2 = (sample_index + 1) % sample_count_per_loop;
            int vert3 = loop_count * sample_count_per_loop + 0;
            //faces[face_index++] = {vert1, vert2, vert3};
            faces.push_back({vert1, vert2, vert3});
        }
        // top side
        for (int sample_index = 0; sample_index < sample_count_per_loop; sample_index++) {
            int vert1 = sample_count_per_loop * (loop_count - 1) + sample_index;
            int vert2 = sample_count_per_loop * (loop_count - 1) + (sample_index + 1) % sample_count_per_loop;
            int vert3 = loop_count * sample_count_per_loop + 1;
            //faces[face_index++] = {vert1, vert2, vert3};
            faces.push_back({vert1, vert2, vert3});
        }
    }
};
};