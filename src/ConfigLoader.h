/*
 * ConfigLoader.h
 *
 *  Created on: 22/lug/2012
 *      Author: Mladen Mazuran
 */

#ifndef CONFIGLOADER_H_
#define CONFIGLOADER_H_

#include <stdarg.h>
#include <ros/ros.h>

/* Macro for quickly loading a configuration parameter */
#define CONFIG_LOAD(name) {                                         \
    __typeof__(config::name) t;                                     \
    if(handle.getParam(paramPrefix + "/" #name, t)) {               \
        config::name = t;                                           \
    }                                                               \
}

/* Macro for loading enums, parameters require value-representation string pairs */
#define CONFIG_LOAD_ENUM(name, count, ...)                          \
        loadConfigEnum(handle, paramPrefix, config::name, #name,    \
                count, __VA_ARGS__);

/* Enum load support function */
template <typename T>
void loadConfigEnum(
        ros::NodeHandle &handle, std::string &paramPrefix,
        T &var, const char *name, int count, ...) {
    std::string t;
    if(handle.getParam(paramPrefix + "/" + name, t)) {
        va_list vl;
        /* The representation string is considered case insensitively */
        std::transform(t.begin(), t.end(), t.begin(), ::tolower);
        va_start(vl, count);
        /* Check value against enumerated ones */
        for(int i = 0; i < count; i++) {
            T value = (T) va_arg(vl, int);
            std::string key = va_arg(vl, const char *);
            /* Case insensitive */
            std::transform(key.begin(), key.end(), key.begin(), ::tolower);
            if(t == key) {
                var = value;
                break;
            }
        }
        va_end(vl);
    }
}

/* Loads the configuration from the ROS parameter server */
static void loadConfig(ros::NodeHandle &handle)
{
    std::string paramPrefix = ros::this_node::getName();

    CONFIG_LOAD(laser_topic)
    CONFIG_LOAD(odometry_topic)

    CONFIG_LOAD(tf_child_frame)
    CONFIG_LOAD(tf_parent_frame)
    CONFIG_LOAD(send_transform)

    CONFIG_LOAD(laser_queue_len)
    CONFIG_LOAD(odometry_queue_len)
    CONFIG_LOAD(max_rate)
    CONFIG_LOAD(sigma2)

    CONFIG_LOAD(icp_metric_l)
    CONFIG_LOAD(icp_trim_ratio)
    CONFIG_LOAD(icp_conv_error)
    CONFIG_LOAD(icp_max_iter)

    CONFIG_LOAD_ENUM(nn_engine, 2,
            config::ON2,            "ON2",
            config::ANN,            "ANN")
    CONFIG_LOAD_ENUM(icp_algorithm, 3,
            config::Classic,        "Classic",
            config::Metric,         "Metric",
            config::PointToLine,    "PointToLine")
}



#endif /* CONFIGLOADER_H_ */
