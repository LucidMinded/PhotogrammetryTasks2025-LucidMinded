#include "sfm_utils.h"

#include <algorithm>
#include <cmath>
#include <opencv2/core/matx.hpp>
#include <stdexcept>

// pseudorandom number generator
uint64_t xorshift64(uint64_t *state) {
    if (*state == 0) {
        *state = 1;
    }

    uint64_t x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return *state = x;
}

void phg::randomSample(std::vector<int> &dst, int max_id, int sample_size, uint64_t *state) {
    dst.clear();

    const int max_attempts = 1000;

    for (int i = 0; i < sample_size; ++i) {
        for (int k = 0; k < max_attempts; ++k) {
            int v = xorshift64(state) % max_id;
            if (dst.empty() || std::find(dst.begin(), dst.end(), v) == dst.end()) {
                dst.push_back(v);
                break;
            }
        }
        if (dst.size() < i + 1) {
            throw std::runtime_error("Failed to sample ids");
        }
    }
}

inline double distToLine(const cv::Vec2d &pt, const cv::Vec3d &line) {
    const double a = line[0], b = line[1], c = line[2];
    return std::abs(a * pt[0] + b * pt[1] + c) / std::sqrt(a * a + b * b);
}

// проверяет, что расстояние от точки до линии меньше порога
bool phg::epipolarTest(const cv::Vec2d &pt0, const cv::Vec2d &pt1, const cv::Matx33d &F, double t) {
    cv::Vec3d pt0h(pt0[0], pt0[1], 1.0);
    cv::Vec3d l1 = F * pt0h;
    return distToLine(pt1, l1) < t;
}
