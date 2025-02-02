//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_MODULE_FRAME_TRACKER_H
#define VSLAM_MODULE_FRAME_TRACKER_H

#include "vslam/type.h"
#include "vslam/optimize/pose_optimizer.h"

namespace vslam {
    namespace camera {
        class base;
    }
    namespace data {
        class frame;

        class keyframe;
    }
    namespace module {
        class frame_tracker {
        public:
            explicit frame_tracker(camera::base *camera, const unsigned int num_matches_thr = 20);

            bool motion_based_track(data::frame &curr_frm, const data::frame &last_frm, const Mat44_t &velocity) const;

            bool
            bow_match_based_track(data::frame &curr_frm, const data::frame &last_frm, data::keyframe *ref_keyfrm) const;

            bool robust_match_based_track(data::frame &curr_frm, const data::frame &last_frm,
                                          data::keyframe *ref_keyfrm) const;

        private:
            unsigned int discard_outliers(data::frame &curr_frm) const;

            const camera::base *camera_;
            const unsigned int num_matches_thr_;

            const optimize::pose_optimizer pose_optimizer_;
        };
    }
}


#endif //VSLAM_MODULE_FRAME_TRACKER_H
