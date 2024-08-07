//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_SOLVE_COMMON_H
#define VSLAM_SOLVE_COMMON_H

#include "vslam/type.h"

#include <vector>

#include <opencv2/core.hpp>

namespace vslam {
    namespace solve {
        void normalize(const std::vector<cv::KeyPoint> &keypts, std::vector<cv::Point2f> &normalized_pts,
                       Mat33_t &transform);
    }
}

#endif //VSLAM_SOLVE_COMMON_H
