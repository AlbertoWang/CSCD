#pragma once
#include <algorithm>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <limits>

namespace zjl {
class Normalizer {
public:
    template<typename P>
    static std::pair<P, P> normalize(std::vector<P>& points) {
        using namespace std;
        if (points.empty()) {
            cerr << "Error: no points" << endl;
            exit(-1);
        }
        const double LD = std::numeric_limits<double>::lowest();
        const double MD = std::numeric_limits<double>::max();
        P max_point(LD, LD, LD);
        P min_point(MD, MD, MD);
        for (const auto& kv : points) {
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
        for (auto& point : points) {
            point = point - center;
            point = point * scaling;
        }
        return {center, diagonal};
    }
};

};