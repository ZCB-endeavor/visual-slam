//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_OPTIMIZE_INTERNAL_SE3_SHOT_VERTEX_H
#define VSLAM_OPTIMIZE_INTERNAL_SE3_SHOT_VERTEX_H

#include "vslam/type.h"

#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/se3quat.h>

namespace vslam {
    namespace optimize {
        namespace internal {
            namespace se3 {
                class shot_vertex final : public g2o::BaseVertex<6, g2o::SE3Quat> {
                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                    shot_vertex();

                    bool read(std::istream &is) override;

                    bool write(std::ostream &os) const override;

                    void setToOriginImpl() override;

                    void oplusImpl(const number_t *update_) override;
                };

                inline shot_vertex::shot_vertex()
                        : g2o::BaseVertex<6, g2o::SE3Quat>() {}

                inline bool shot_vertex::shot_vertex::read(std::istream &is) {
                    Vec7_t estimate;
                    for (unsigned int i = 0; i < 7; ++i) {
                        is >> estimate(i);
                    }
                    g2o::SE3Quat g2o_cam_pose_wc;
                    g2o_cam_pose_wc.fromVector(estimate);
                    setEstimate(g2o_cam_pose_wc.inverse());
                    return true;
                }

                inline bool shot_vertex::shot_vertex::write(std::ostream &os) const {
                    g2o::SE3Quat g2o_cam_pose_wc(estimate().inverse());
                    for (unsigned int i = 0; i < 7; ++i) {
                        os << g2o_cam_pose_wc[i] << " ";
                    }
                    return os.good();
                }

                inline void shot_vertex::setToOriginImpl() {
                    _estimate = g2o::SE3Quat();
                }

                inline void shot_vertex::oplusImpl(const number_t *update_) {
                    Eigen::Map<const Vec6_t> update(update_);
                    setEstimate(g2o::SE3Quat::exp(update) * estimate());
                }
            }
        }
    }
}

#endif //VSLAM_OPTIMIZE_INTERNAL_SE3_SHOT_VERTEX_H
