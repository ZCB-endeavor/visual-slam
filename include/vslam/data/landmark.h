//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_DATA_LANDMARK_H
#define VSLAM_DATA_LANDMARK_H

#include "vslam/type.h"

#include <map>
#include <mutex>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <nlohmann/json_fwd.hpp>

namespace vslam{
    namespace data{
        class frame;

        class keyframe;

        class map_database;

        class landmark {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            //! constructor
            landmark(const Vec3_t& pos_w, keyframe* ref_keyfrm, map_database* map_db);

            //! constructor for map loading with computing parameters which can be recomputed
            landmark(const unsigned int id, const unsigned int first_keyfrm_id,
                     const Vec3_t& pos_w, keyframe* ref_keyfrm,
                     const unsigned int num_visible, const unsigned int num_found,
                     map_database* map_db);

            //! set world coordinates of this landmark
            void set_pos_in_world(const Vec3_t& pos_w);
            //! get world coordinates of this landmark
            Vec3_t get_pos_in_world() const;

            //! get mean normalized vector of keyframe->lm vectors, for keyframes such that observe the 3D point.
            Vec3_t get_obs_mean_normal() const;
            //! get reference keyframe, a keyframe at the creation of a given 3D point
            keyframe* get_ref_keyframe() const;

            //! add observation
            void add_observation(keyframe* keyfrm, unsigned int idx);
            //! erase observation
            void erase_observation(keyframe* keyfrm);

            //! get observations (keyframe and keypoint idx)
            std::map<keyframe*, unsigned int> get_observations() const;
            //! get number of observations
            unsigned int num_observations() const;
            //! whether this landmark is observed from more than zero keyframes
            bool has_observation() const;

            //! get index of associated keypoint in the specified keyframe
            int get_index_in_keyframe(keyframe* keyfrm) const;
            //! whether this landmark is observed in the specified keyframe
            bool is_observed_in_keyframe(keyframe* keyfrm) const;

            //! check the distance between landmark and camera is in ORB scale variance
            inline bool is_inside_in_orb_scale(const float cam_to_lm_dist) const {
                const float max_dist = this->get_max_valid_distance();
                const float min_dist = this->get_min_valid_distance();
                return (min_dist <= cam_to_lm_dist && cam_to_lm_dist <= max_dist);
            }

            //! get representative descriptor
            cv::Mat get_descriptor() const;

            //! compute representative descriptor
            void compute_descriptor();

            //! update observation mean normal and ORB scale variance
            void update_normal_and_depth();

            //! get max valid distance between landmark and camera
            float get_min_valid_distance() const;
            //! get min valid distance between landmark and camera
            float get_max_valid_distance() const;

            //! predict scale level assuming this landmark is observed in the specified frame
            unsigned int predict_scale_level(const float cam_to_lm_dist, const frame* frm) const;
            //! predict scale level assuming this landmark is observed in the specified keyframe
            unsigned int predict_scale_level(const float cam_to_lm_dist, const keyframe* keyfrm) const;

            //! erase this landmark from database
            void prepare_for_erasing();
            //! whether this landmark will be erased shortly or not
            bool will_be_erased();

            //! replace this with specified landmark
            void replace(landmark* lm);
            //! get replace landmark
            landmark* get_replaced() const;

            void increase_num_observable(unsigned int num_observable = 1);
            void increase_num_observed(unsigned int num_observed = 1);
            float get_observed_ratio() const;

            //! encode landmark information as JSON
            nlohmann::json to_json() const;

        public:
            unsigned int id_;
            static std::atomic<unsigned int> next_id_;
            unsigned int first_keyfrm_id_ = 0;
            unsigned int num_observations_ = 0;

            // Variables for frame tracking.
            Vec2_t reproj_in_tracking_;
            float x_right_in_tracking_;
            bool is_observable_in_tracking_;
            int scale_level_in_tracking_;
            unsigned int identifier_in_local_map_update_ = 0;
            unsigned int identifier_in_local_lm_search_ = 0;

            // Variables for loop-closing.
            unsigned int loop_fusion_identifier_ = 0;
            unsigned int ref_keyfrm_id_in_loop_fusion_ = 0;
            Vec3_t pos_w_after_global_BA_;
            unsigned int loop_BA_identifier_ = 0;

        private:
            //! world coordinates of this landmark
            Vec3_t pos_w_;

            //! observations (keyframe and keypoint index)
            std::map<keyframe*, unsigned int> observations_;

            //! Normalized average vector (unit vector) of keyframe->lm, for keyframes such that observe the 3D point.
            Vec3_t mean_normal_ = Vec3_t::Zero();

            //! representative descriptor
            cv::Mat descriptor_;

            //! reference keyframe
            keyframe* ref_keyfrm_;

            // track counter
            unsigned int num_observable_ = 1;
            unsigned int num_observed_ = 1;

            //! this landmark will be erased shortly or not
            bool will_be_erased_ = false;

            //! replace this landmark with below
            landmark* replaced_ = nullptr;

            // ORB scale variances
            //! max valid distance between landmark and camera
            float min_valid_dist_ = 0;
            //! min valid distance between landmark and camera
            float max_valid_dist_ = 0;

            //! map database
            map_database* map_db_;

            mutable std::mutex mtx_position_;
            mutable std::mutex mtx_observations_;
        };
    }
}

#endif //VSLAM_DATA_LANDMARK_H
