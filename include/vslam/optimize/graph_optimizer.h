//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H
#define VSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H

#include "vslam/module/type.h"

#include <map>
#include <set>

namespace vslam {
    namespace data {
        class keyframe;

        class map_database;
    }
    namespace optimize {
        class graph_optimizer {
        public:
            /**
             * Constructor
             * @param map_db
             * @param fix_scale
             */
            explicit graph_optimizer(data::map_database *map_db, const bool fix_scale);

            /**
             * Destructor
             */
            virtual ~graph_optimizer() = default;

            /**
             * Perform pose graph optimization
             * @param loop_keyfrm
             * @param curr_keyfrm
             * @param non_corrected_Sim3s
             * @param pre_corrected_Sim3s
             * @param loop_connections
             */
            void optimize(data::keyframe *loop_keyfrm, data::keyframe *curr_keyfrm,
                          const module::keyframe_Sim3_pairs_t &non_corrected_Sim3s,
                          const module::keyframe_Sim3_pairs_t &pre_corrected_Sim3s,
                          const std::map<data::keyframe *, std::set<data::keyframe *>> &loop_connections) const;

        private:
            //! map database
            const data::map_database *map_db_;

            //! SE3 optimization or Sim3 optimization
            const bool fix_scale_;
        };
    }
}

#endif //VSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H
