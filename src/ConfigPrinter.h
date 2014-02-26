/*
 * ConfigPrinter.h
 *
 *  Created on: 01/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef CONFIGPRINTER_H_
#define CONFIGPRINTER_H_
#include <sstream>

/* Macro for quickly printing the configuration */
#define CONFIG_PRINT(name) {                                        \
    std::stringstream ss;                                           \
    ss << std::setw(18) << #name << ": " << config::name;           \
    ROS_INFO("%s", ss.str().c_str());                               \
}

/* Pretty format of the nearest neighbour engine */
static std::ostream &operator<<(std::ostream &stream, config::NNEngineType nn) {
    switch(nn) {
    case config::ANN: stream << "ANN"; break;
    case config::ON2: stream << "ON2"; break;
    }
    return stream;
}

/* Pretty format of the ICP algorithm */
static std::ostream &operator<<(std::ostream &stream, config::ICPAlgorithmType icp) {
    switch(icp) {
    case config::Classic:     stream << "Classic";      break;
    case config::Metric:      stream << "Metric";       break;
    case config::PointToLine: stream << "PointToLine";  break;
    }
    return stream;
}

/* Prints all of the loaded configuration */
static void printConfig() {
    ROS_INFO("Node configuration:");
    CONFIG_PRINT(laser_topic)
    CONFIG_PRINT(odometry_topic)

    CONFIG_PRINT(tf_child_frame)
    CONFIG_PRINT(tf_parent_frame)
    CONFIG_PRINT(send_transform)

    CONFIG_PRINT(laser_queue_len)
    CONFIG_PRINT(odometry_queue_len)
    CONFIG_PRINT(max_rate)
    CONFIG_PRINT(sigma2)

    CONFIG_PRINT(icp_metric_l)
    CONFIG_PRINT(icp_trim_ratio)
    CONFIG_PRINT(icp_conv_error)
    CONFIG_PRINT(icp_max_iter)

    CONFIG_PRINT(nn_engine)
    CONFIG_PRINT(icp_algorithm)

}


#endif /* CONFIGPRINTER_H_ */
