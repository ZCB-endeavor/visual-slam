//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_CONFIG_H
#define VSLAM_CONFIG_H

#include "vslam/camera/base.h"
#include "vslam/feature/orb_params.h"

#include <yaml-cpp/yaml.h>

namespace vslam {
    class config {
    public:
        //! Constructor
        explicit config(const std::string &config_file_path);

        explicit config(const YAML::Node &yaml_node, const std::string &config_file_path = "");

        //! Destructor
        ~config();

        friend std::ostream &operator<<(std::ostream &os, const config &cfg);

        //! path to config YAML file
        const std::string config_file_path_;

        //! YAML node
        const YAML::Node yaml_node_;

        //! Camera model
        camera::base *camera_ = nullptr;
    };
}

#endif //VSLAM_CONFIG_H
