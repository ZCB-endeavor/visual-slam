//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_OPTIMIZE_POSE_OPTIMIZER_H
#define VSLAM_OPTIMIZE_POSE_OPTIMIZER_H

namespace vslam {
    namespace data {
        class frame;
    }
    namespace optimize {
        class pose_optimizer {
        public:
            /**
             * Constructor
             * @param num_trials
             * @param num_each_iter
             */
            explicit pose_optimizer(const unsigned int num_trials = 4, const unsigned int num_each_iter = 10);

            /**
             * Destructor
             */
            virtual ~pose_optimizer() = default;

            /**
             * Perform pose optimization
             * @param frm
             * @return
             */
            unsigned int optimize(data::frame &frm) const;

        private:
            //! robust optimizationの試行回数
            const unsigned int num_trials_ = 4;

            //! 毎回のoptimizationのiteration回数
            const unsigned int num_each_iter_ = 10;
        };
    }
}

#endif //VSLAM_OPTIMIZE_POSE_OPTIMIZER_H
