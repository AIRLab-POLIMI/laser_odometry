/*
 * Conversions.h
 *
 *  Created on: 22/lug/2012
 *      Author: Mladen Mazuran
 */

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include "Macros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

namespace conversions {

static const double MAXIMUM_SPATIAL_UNCERTAINTY = 1e6;
static const double MAXIMUM_ANGULAR_UNCERTAINTY = 4 * M_PI * M_PI;
static const double MAXIMUM_TWIST_UNCERTAINTY = 1e6;

/* Converts a ROS laser message to the internal object format */
static Scan *rosToScan(const sensor_msgs::LaserScan::ConstPtr& s) {
    Scan *scan = new Scan();
    float angle = s->angle_min;
    foreach(float range, s->ranges) {
        if(range > 1e-4)
            scan->push_back(Point::Polar(range, angle));
        angle += s->angle_increment;
    }
    return scan;
}

/* Converts a planar pose to a ROS transform */
static void vectorToROSTransform (
        const Eigen::Vector3d &pose, geometry_msgs::Transform &ros) {
    ros.translation.x = pose[0];
    ros.translation.y = pose[1];
    ros.translation.z = 0;

    ros.rotation = tf::createQuaternionMsgFromYaw(pose[2]);
}

/* Converts a planar pose to a ROS pose message */
static void vectorToROSPose(
        const Eigen::Vector3d &pose, geometry_msgs::Pose &ros) {
    ros.position.x = pose[0];
    ros.position.y = pose[1];
    ros.position.z = 0;

    ros.orientation = tf::createQuaternionMsgFromYaw(pose[2]);
}

/* Flattens a 2d index to the 1d indexing in a 6x6 matrix */
static inline int flatten(int i, int j) {
    return i * 6 + j;
}

/* Converts a planar 3x3 pose covariance matrix to the ROS matrix format */
static void covarianceToROS6DOF(
        const Eigen::Matrix3d &cov, boost::array<double, 36> &ros) {
    /* Initialize everything so that the unset off diagonal components are zero */
    for(int i = 0; i < 36; i++) {
        ros[i] = 0;
    }

    ros[flatten(0,0)] = cov(0,0);
    ros[flatten(1,1)] = cov(1,1);
    ros[flatten(5,5)] = cov(2,2);

    ros[flatten(0,1)] = cov(0,1);
    ros[flatten(1,0)] = cov(1,0);

    ros[flatten(0,5)] = cov(0,2);
    ros[flatten(5,0)] = cov(2,0);

    ros[flatten(1,5)] = cov(1,2);
    ros[flatten(5,1)] = cov(2,1);

    /* The remaining 3dof are unknown */
    ros[flatten(2,2)] = MAXIMUM_SPATIAL_UNCERTAINTY;
    ros[flatten(3,3)] = MAXIMUM_ANGULAR_UNCERTAINTY;
    ros[flatten(4,4)] = MAXIMUM_ANGULAR_UNCERTAINTY;

}

/* Converts a twist (angular velocity) vector to a ROS message */
static void twistToROS6DOF(
        const Eigen::Vector3d &twist, geometry_msgs::Twist &ros) {
    ros.linear.x = twist[0];
    ros.linear.y = twist[1];
    ros.linear.z = 0;

    ros.angular.x = 0;
    ros.angular.y = 0;
    ros.angular.z = twist[2];
}

/* Converts a twist covariance matrix to the ROS matrix format */
static void twistCovarianceToROS6DOF(
        const Eigen::Matrix3d &cov, boost::array<double, 36> &ros) {

    covarianceToROS6DOF(cov, ros);

    ros[flatten(2,2)] = MAXIMUM_TWIST_UNCERTAINTY;
    ros[flatten(3,3)] = MAXIMUM_TWIST_UNCERTAINTY;
    ros[flatten(4,4)] = MAXIMUM_TWIST_UNCERTAINTY;
}

} /* namespace conversions */

#endif /* CONVERSIONS_H_ */
