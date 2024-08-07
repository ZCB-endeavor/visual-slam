//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_DATA_FRAME_STATISTICS_H
#define VSLAM_DATA_FRAME_STATISTICS_H

#include "vslam/type.h"

#include <vector>
#include <unordered_map>

namespace vslam {
    namespace data {
        class frame;

        class keyframe;

        class frame_statistics {
        public:
            /**
             * Constructor
             */
            frame_statistics() = default;

            /**
             * Destructor
             */
            virtual ~frame_statistics() = default;

            /**
             * Update frame statistics
             * @param frm
             * @param is_lost
             */
            void update_frame_statistics(const data::frame &frm, const bool is_lost);

            /**
             * Replace a keyframe which will be erased in frame statistics
             * @param old_keyfrm
             * @param new_keyfrm
             */
            void replace_reference_keyframe(data::keyframe *old_keyfrm, data::keyframe *new_keyfrm);

            /**
             * Get frame IDs of each of the reference keyframes
             * @return
             */
            std::unordered_map<data::keyframe *, std::vector<unsigned int>> get_frame_id_of_reference_keyframes() const;

            /**
             * Get the number of the contained valid frames
             * @return
             */
            unsigned int get_num_valid_frames() const;

            /**
             * Get reference keyframes of each of the frames
             * @return
             */
            std::map<unsigned int, data::keyframe *> get_reference_keyframes() const;

            /**
             * Get relative camera poses from the corresponding reference keyframes
             * @return
             */
            eigen_alloc_map<unsigned int, Mat44_t> get_relative_cam_poses() const;

            /**
             * Get timestamps
             * @return
             */
            std::map<unsigned int, double> get_timestamps() const;

            std::map<unsigned int, std::string> get_image_name() const;

            /**
             * Get lost frame flags
             * @return
             */
            std::map<unsigned int, bool> get_lost_frames() const;

            /**
             * Clear frame statistics
             */
            void clear();

        private:
            //! Reference keyframe, frame ID associated with the keyframe
            std::unordered_map<data::keyframe *, std::vector<unsigned int>> frm_ids_of_ref_keyfrms_;

            //! Number of valid frames
            unsigned int num_valid_frms_ = 0;

            // Size of all the following variables is the number of frames
            //! Reference keyframes for each frame
            std::unordered_map<unsigned int, data::keyframe *> ref_keyfrms_;
            //! Relative pose against reference keyframe for each frame
            eigen_alloc_unord_map<unsigned int, Mat44_t> rel_cam_poses_from_ref_keyfrms_;
            //! Timestamp for each frame
            std::unordered_map<unsigned int, double> timestamps_;
            //! Image Name for each frame
            std::unordered_map<unsigned int, std::string> img_name_;
            //! Flag whether each frame is lost or not
            std::unordered_map<unsigned int, bool> is_lost_frms_;
        };
    }
}

#endif //VSLAM_DATA_FRAME_STATISTICS_H
