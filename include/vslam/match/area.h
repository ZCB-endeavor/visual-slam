//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_MATCH_AREA_H
#define VSLAM_MATCH_AREA_H

#include "vslam/match/base.h"

namespace vslam {
    namespace data {
        class frame;
    }
    namespace match {
        class area final : public base {
        public:
            area(const float lowe_ratio, const bool check_orientation)
                    : base(lowe_ratio, check_orientation) {}

            ~area() final = default;

            unsigned int
            match_in_consistent_area(data::frame &frm_1, data::frame &frm_2, std::vector<cv::Point2f> &prev_matched_pts,
                                     std::vector<int> &matched_indices_2_in_frm_1, int margin = 10);
        };
    }
}

#endif //VSLAM_MATCH_AREA_H
