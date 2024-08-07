//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_SYSTEM_H
#define VSLAM_SYSTEM_H

#include "vslam/type.h"
#include "vslam/data/bow_vocabulary_fwd.h"

#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/core/core.hpp>

namespace vslam {
    class config;

    class tracking_module;

    class mapping_module;

    class global_optimization_module;
    namespace camera {
        class base;
    }
    namespace data {
        class camera_database;

        class map_database;

        class bow_database;
    }
    namespace publish {
        class map_publisher;

        class frame_publisher;
    }
    class system {
    public:
        //! 构造函数
        system(const std::shared_ptr<config> &cfg, const std::string &vocab_file_path);

        //! 析构函数
        ~system();

        //! 启动SLAM系统
        void startup(const bool need_initialize = true);

        //! 关闭SLAM系统
        void shutdown();

        //! 使用特定格式保存轨迹
        void save_frame_trajectory(const std::string &path, const std::string &format) const;

        //! 使用特定格式保存关键帧轨迹
        void save_keyframe_trajectory(const std::string &path, const std::string &format) const;

        //! 保存地图数据库到msg文件
        void save_map_database(const std::string &path) const;

        //! 获取地图信息
        const std::shared_ptr<publish::map_publisher> get_map_publisher() const;

        //! 获取帧信息
        const std::shared_ptr<publish::frame_publisher> get_frame_publisher() const;

        //! 使能建图模块
        void enable_mapping_module();

        //! 禁用建图模块
        void disable_mapping_module();

        //! 建图模块状态
        bool mapping_module_is_enabled() const;

        //! 使能回环检测
        void enable_loop_detector();

        //! 禁用回环检测
        void disable_loop_detector();

        //! 回环检测状态
        bool loop_detector_is_enabled() const;

        //! 循环BA是否运行
        bool loop_BA_is_running() const;

        //! 外部终止循环BA
        //void abort_loop_BA();

        //! 加载单目图像到SLAM系统
        std::shared_ptr<Mat44_t>
        feed_monocular_frame(const std::string image_name, const cv::Mat &img, const double timestamp,
                             const cv::Mat &mask = cv::Mat{});

        //! 请求更新位姿
        //bool update_pose(const Mat44_t &pose);

        //! 暂停跟踪模块
        void pause_tracker();

        //! 跟踪模块状态
        bool tracker_is_paused() const;

        //! 恢复跟踪模块
        void resume_tracker();

        //! 重置系统
        void request_reset();

        //! 重置系统是否请求
        //bool reset_is_requested() const;

        //! 请求终止系统
        void request_terminate();

        //!! 终止系统是否请求
        bool terminate_is_requested() const;

    private:
        //! 检查重置请求状态
        void check_reset_request();

        //! 暂停建图模块与全局优化模块
        void pause_other_threads() const;

        //! 恢复建图模块与全局优化模块
        void resume_other_threads() const;

        //! 配置
        const std::shared_ptr<config> cfg_;
        //! 相机模型
        camera::base *camera_ = nullptr;

        //! 相机数据库
        data::camera_database *cam_db_ = nullptr;

        //! 地图数据库
        data::map_database *map_db_ = nullptr;

        //! BoW词库
        data::bow_vocabulary *bow_vocab_ = nullptr;

        //! BoW数据库
        data::bow_database *bow_db_ = nullptr;

        //! 跟踪器
        tracking_module *tracker_ = nullptr;

        //! 建图模块
        mapping_module *mapper_ = nullptr;
        //! 建图线程
        std::unique_ptr<std::thread> mapping_thread_ = nullptr;

        //! 全局优化模块
        global_optimization_module *global_optimizer_ = nullptr;
        //! 全局优化线程
        std::unique_ptr<std::thread> global_optimization_thread_ = nullptr;

        //! 帧数据
        std::shared_ptr<publish::frame_publisher> frame_publisher_ = nullptr;
        //! 地图数据
        std::shared_ptr<publish::map_publisher> map_publisher_ = nullptr;

        //! 系统运行状态
        std::atomic<bool> system_is_running_{false};

        //! 互斥的重置状态
        mutable std::mutex mtx_reset_;
        //! 重置请求状态
        bool reset_is_requested_ = false;

        //! 互斥的终止状态
        mutable std::mutex mtx_terminate_;
        //! 终止请求状态
        bool terminate_is_requested_ = false;

        //! 互斥的建图模块状态
        mutable std::mutex mtx_mapping_;

        //! 互斥的回环检测状态
        mutable std::mutex mtx_loop_detector_;
    };
}

#endif //VSLAM_SYSTEM_H
