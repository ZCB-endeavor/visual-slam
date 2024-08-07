//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_TRACKING_MODULE_H
#define VSLAM_TRACKING_MODULE_H

#include "vslam/type.h"
#include "vslam/data/frame.h"
#include "vslam/module/initializer.h"
#include "vslam/module/relocalizer.h"
#include "vslam/module/keyframe_inserter.h"
#include "vslam/module/frame_tracker.h"

#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace vslam {
    class system;

    class mapping_module;

    class global_optimization_module;
    namespace data {
        class map_database;

        class bow_database;
    }
    namespace feature {
        class orb_extractor;
    }
    enum class tracker_state_t {
        NotInitialized,
        Initializing,
        Tracking,
        Lost
    };

    class tracking_module {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! 构造函数
        tracking_module(const std::shared_ptr<config> &cfg, system *system, data::map_database *map_db,
                        data::bow_vocabulary *bow_vocab, data::bow_database *bow_db);

        //! 析构函数
        ~tracking_module();

        //! 设置地图模块
        void set_mapping_module(mapping_module *mapper);

        //! 设置全局优化模块
        void set_global_optimization_module(global_optimization_module *global_optimizer);

        //! 设置地图模块状态
        void set_mapping_module_status(const bool mapping_is_enabled);

        //! 获取地图模块状态
        bool get_mapping_module_status() const;

        //! 获取初始帧关键点
        std::vector<cv::KeyPoint> get_initial_keypoints() const;

        //! 获取与初始帧匹配的当前帧的关键点
        std::vector<int> get_initial_matches() const;

        //! 跟踪单目图像序列
        std::shared_ptr<Mat44_t>
        track_monocular_image(const std::string image_name, const cv::Mat &img, const double timestamp,
                              const cv::Mat &mask = cv::Mat{});

        //! 更新当前帧的位姿 若之前请求未结束 返回false
        bool request_update_pose(const Mat44_t &pose);

        //! 重置数据库
        void reset();

        //! 暂停跟踪模块
        void request_pause();

        //! 检查是否请求跟踪模块暂停
        //bool pause_is_requested() const;

        //! 检查跟踪模块是否暂停
        bool is_paused() const;

        //! 恢复跟踪
        void resume();

        //! 相机模型
        camera::base *camera_;

        //! 深度阈值
        double true_depth_thr_ = 40.0;

        //! 深度图元素
        double depthmap_factor_ = 1.0;

        //! 相近关键帧阈值
        double reloc_distance_threshold_ = 0.2;
        double reloc_angle_threshold_ = 0.45;

        //! 最终跟踪状态
        tracker_state_t tracking_state_ = tracker_state_t::NotInitialized;
        //! 上次跟踪状态
        tracker_state_t last_tracking_state_ = tracker_state_t::NotInitialized;

        //! 当前帧
        data::frame curr_frm_;
        //! 当前帧图像
        cv::Mat img_gray_;
        cv::Mat img_color_;

        //! 跟踪时间
        double elapsed_ms_ = 0.0;

    protected:
        //! 跟踪模块主要数据流
        void track();

        //! 初始化
        bool initialize();

        //! 跟踪当前帧
        bool track_current_frame();

        //! 使用上一帧与当前帧更新显示视图
        void update_motion_model();

        //! 替换地标
        void apply_landmark_replace();

        //! 更新上一帧相机位姿
        void update_last_frame();

        //! 优化当前帧相机位姿
        bool optimize_current_frame_with_local_map();

        //! 更新局部地图
        void update_local_map();

        //! 查找局部地标进行位姿估计
        void search_local_landmarks();

        //! 检查新关键帧是否需要
        bool new_keyframe_is_needed() const;

        //! 插入新关键帧
        void insert_new_keyframe();

        //! 系统模块
        system *system_ = nullptr;
        //! 地图模块
        mapping_module *mapper_ = nullptr;
        //! 全局优化模块
        global_optimization_module *global_optimizer_ = nullptr;

        //! 参考帧ORB特征提取
        feature::orb_extractor *extractor_left_ = nullptr;
        //! 当前帧ORB特征提取
        feature::orb_extractor *extractor_right_ = nullptr;
        //! 初始化ORB特征提取
        feature::orb_extractor *ini_extractor_left_ = nullptr;

        //! 地图数据库
        data::map_database *map_db_ = nullptr;

        //! 回环检测词库
        data::bow_vocabulary *bow_vocab_ = nullptr;
        //! 回环检测数据库
        data::bow_database *bow_db_ = nullptr;

        //! 初始化模块
        module::initializer initializer_;

        //! 当前帧跟踪模块
        const module::frame_tracker frame_tracker_;

        //! 重定位模块
        module::relocalizer relocalizer_;

        //! 位姿优化模块
        const optimize::pose_optimizer pose_optimizer_;

        //! 插入的关键帧
        module::keyframe_inserter keyfrm_inserter_;

        //! 局部关键帧
        std::vector<data::keyframe *> local_keyfrms_;
        //! 局部地标
        std::vector<data::landmark *> local_landmarks_;

        //! 当前跟踪关键帧的地标数量
        unsigned int num_tracked_lms_ = 0;

        //! 上一帧
        data::frame last_frm_;

        //! 最新重定位成功的帧ID
        unsigned int last_reloc_frm_id_ = 0;

        //! 运动模型
        Mat44_t twist_;
        //! 运动模型是否有效
        bool twist_is_valid_ = false;

        //! 从参考关键帧估计当前相机姿态
        Mat44_t last_cam_pose_from_ref_keyfrm_;

        //! 互斥的建图模块状态
        mutable std::mutex mtx_mapping_;

        //! 建图模块是否有效
        bool mapping_is_enabled_ = true;

        //! 互斥的暂停进程
        mutable std::mutex mtx_pause_;

        //! 检查是否请求帧或是暂停跟踪模块
        bool check_and_execute_pause();

        //! 跟踪模块是否暂停
        bool is_paused_ = false;

        //! 暂停跟踪模块是否请求
        bool pause_is_requested_ = false;

        //! 互斥的更新位姿请求
        mutable std::mutex mtx_update_pose_request_;

        //! 是否请求更新位姿
        bool update_pose_is_requested();

        //! 获取重定位位姿
        Mat44_t &get_requested_pose();

        //! 完成更新位姿请求
        void finish_update_pose_request();

        //! 更新位姿请求标志
        bool update_pose_is_requested_ = false;
        //! 请求更新位姿
        Mat44_t requested_pose_;
    };
}


#endif //VSLAM_TRACKING_MODULE_H
