//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_FEATURE_ORB_PARAMS_H
#define VSLAM_FEATURE_ORB_PARAMS_H

#include <yaml-cpp/yaml.h>

namespace vslam {
    namespace feature {
        struct orb_params {
            orb_params() = default;

            //! Constructor
            orb_params(const unsigned int max_num_keypts, const unsigned int ini_max_num_keypts,
                       const float scale_factor, const unsigned int num_levels,
                       const unsigned int ini_fast_thr, const unsigned int min_fast_thr,
                       const std::vector<std::vector<float>> &mask_rects = {});

            //! Constructor
            explicit orb_params(const YAML::Node &yaml_node);

            //! Destructor
            virtual ~orb_params() = default;

            unsigned int max_num_keypts_ = 2000;
            unsigned int ini_max_num_keypts_ = 4000;
            float scale_factor_ = 1.2;
            unsigned int num_levels_ = 8;
            unsigned int ini_fast_thr_ = 20;
            unsigned int min_fast_thr = 7;

            //! A vector of keypoint area represents mask area
            //! Each areas are denoted as form of [x_min / cols, x_max / cols, y_min / rows, y_max / rows]
            std::vector<std::vector<float>> mask_rects_;

            //! Calculate scale factors
            static std::vector<float> calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

            //! Calculate inverses of scale factors
            static std::vector<float>
            calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

            //! Calculate squared sigmas at all levels
            static std::vector<float>
            calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

            //! Calculate inverses of squared sigmas at all levels
            static std::vector<float>
            calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);
        };

        std::ostream &operator<<(std::ostream &os, const orb_params &oparam);

    }
}

#endif //VSLAM_FEATURE_ORB_PARAMS_H
