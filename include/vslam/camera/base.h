//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_CAMERA_BASE_H
#define VSLAM_CAMERA_BASE_H

#include "vslam/type.h"

#include <string>
#include <limits>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json_fwd.hpp>

namespace vslam {
    namespace camera {
        enum class setup_type_t {
            Monocular = 0
        };

        const std::array<std::string, 3> setup_type_to_string = {{"Monocular"}};

        enum class model_type_t {
            Perspective = 0
        };

        const std::array<std::string, 4> model_type_to_string = {{"Perspective"}};

        enum class color_order_t {
            Gray = 0,
            RGB = 1,
            BGR = 2
        };

        const std::array<std::string, 3> color_order_to_string = {{"Gray", "RGB", "BGR"}};

        struct image_bounds {
            //! Default constructor
            image_bounds() = default;

            //! Constructor with initialization
            template<typename T, typename U>
            image_bounds(const T min_x, const U max_x, const T min_y, const U max_y)
                    : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y) {}

            float min_x_ = 0.0;
            float max_x_ = 0.0;
            float min_y_ = 0.0;
            float max_y_ = 0.0;
        };

        class base {
        public:
            //! Constructor
            base(const std::string &name, const setup_type_t setup_type, const model_type_t model_type,
                 const color_order_t color_order,
                 const unsigned int cols, const unsigned int rows, const double fps,
                 const double focal_x_baseline, const double true_baseline,
                 const unsigned int num_grid_cols = 64, const unsigned int num_grid_rows = 48);

            //! Destructor
            virtual ~base();

            //! camera name
            const std::string name_;

            //! setup type
            const setup_type_t setup_type_;

            //! Get setup type as string
            std::string get_setup_type_string() const {
                return setup_type_to_string.at(static_cast<unsigned int>(setup_type_));
            }

            //! Load setup type from YAML
            static setup_type_t load_setup_type(const YAML::Node &yaml_node);

            //! Load setup type from string
            static setup_type_t load_setup_type(const std::string &setup_type_str);

            //! model type
            const model_type_t model_type_;

            //! Get model type as string
            std::string get_model_type_string() const {
                return model_type_to_string.at(static_cast<unsigned int>(model_type_));
            }

            //! Load model type from YAML
            static model_type_t load_model_type(const YAML::Node &yaml_node);

            //! Load model type from string
            static model_type_t load_model_type(const std::string &model_type_str);

            //! color order
            const color_order_t color_order_;

            //! Get color order as string
            std::string get_color_order_string() const {
                return color_order_to_string.at(static_cast<unsigned int>(color_order_));
            }

            //! Load color order from YAML
            static color_order_t load_color_order(const YAML::Node &yaml_node);

            //! Load color order from string
            static color_order_t load_color_order(const std::string &color_order_str);

            //! Show common parameters along camera models
            void show_common_parameters() const;

            //---------------------------
            // To be set in the base class

            //! width of image
            const unsigned int cols_;
            //! height of image
            const unsigned int rows_;

            //! frame rate of image
            const double fps_;

            //! focal x true baseline
            const double focal_x_baseline_;

            //! true baseline length in metric scale
            const double true_baseline_;

            //! number of columns of grid to accelerate reprojection matching
            const unsigned int num_grid_cols_;
            //! number of rows of grid to accelerate reprojection matching
            const unsigned int num_grid_rows_;

            //---------------------------
            // To be set in derived classes

            //! information of image boundary
            image_bounds img_bounds_;

            //! cell width of grid pattern
            double inv_cell_width_ = std::numeric_limits<double>::quiet_NaN();
            //! cell height of grid pattern
            double inv_cell_height_ = std::numeric_limits<double>::quiet_NaN();

            //-------------------------
            // To be implemented in derived classes

            //! Show camera parameters
            virtual void show_parameters() const = 0;

            //! Compute image boundaries according to camera model
            virtual image_bounds compute_image_bounds() const = 0;

            //! Undistort keypoint according to camera model
            virtual cv::KeyPoint undistort_keypoint(const cv::KeyPoint &dist_keypt) const = 0;

            //! Undistort keypoints according to camera model
            virtual void undistort_keypoints(const std::vector<cv::KeyPoint> &dist_keypts,
                                             std::vector<cv::KeyPoint> &undist_keypts) const = 0;

            //! Convert undistorted keypoint to bearing vector
            virtual Vec3_t convert_keypoint_to_bearing(const cv::KeyPoint &undist_keypt) const = 0;

            //! Convert undistorted keypoints to bearing vectors
            virtual void convert_keypoints_to_bearings(const std::vector<cv::KeyPoint> &undist_keypts,
                                                       eigen_alloc_vector<Vec3_t> &bearings) const = 0;

            //! Convert bearing vector to undistorted keypoint
            virtual cv::KeyPoint convert_bearing_to_keypoint(const Vec3_t &bearing) const = 0;

            //! Convert bearing vectors to undistorted keypoints
            virtual void convert_bearings_to_keypoints(const eigen_alloc_vector<Vec3_t> &bearings,
                                                       std::vector<cv::KeyPoint> &undist_keypts) const = 0;

            //! Reproject the specified 3D point to image using camera pose and projection model
            //! (reprojected to inside of image -> true, to outside of image -> false)
            virtual bool
            reproject_to_image(const Mat33_t &rot_cw, const Vec3_t &trans_cw, const Vec3_t &pos_w, Vec2_t &reproj,
                               float &x_right) const = 0;

            //! Reproject the specified 3D point to bearing vector using camera pose (Not depends on any projection models)
            //! (reprojected to inside of image -> true, to outside of image -> false)
            virtual bool reproject_to_bearing(const Mat33_t &rot_cw, const Vec3_t &trans_cw, const Vec3_t &pos_w,
                                              Vec3_t &reproj) const = 0;

            //! Encode camera information as JSON
            virtual nlohmann::json to_json() const = 0;
        };

        std::ostream &operator<<(std::ostream &os, const base &params);

    }
}


#endif //VSLAM_CAMERA_BASE_H
