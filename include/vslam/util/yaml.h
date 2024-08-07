//
// Created by zcb on 2022/4/14.
//

#ifndef VSLAM_UTIL_YAML_H
#define VSLAM_UTIL_YAML_H

#include <string>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

namespace vslam {
    namespace util {
        inline YAML::Node yaml_optional_ref(const YAML::Node &ref_node, const std::string &key) {
            return ref_node[key] ? ref_node[key] : YAML::Node();
        }
    }
}

#endif //VSLAM_UTIL_YAML_H
