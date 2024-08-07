//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_UTIL_IMAGE_CONVERTER_H
#define VSLAM_UTIL_IMAGE_CONVERTER_H

#include "vslam/camera/base.h"

#include <opencv2/core.hpp>

namespace vslam {
    namespace util {
        void convert_to_grayscale(cv::Mat &img, const camera::color_order_t in_color_order);

        void convert_to_true_depth(cv::Mat &img, const double depthmap_factor);

        void equalize_histogram(cv::Mat &img);
    }
}

#endif //VSLAM_UTIL_IMAGE_CONVERTER_H
