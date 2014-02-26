/*
 * LaserOdometryNode.h
 *
 *  Created on: 21/lug/2012
 *      Author: Mladen Mazuran
 */

#ifndef LASERODOMETRYNODE_H_
#define LASERODOMETRYNODE_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "scanmatching/Scan.h"

class ICP;

class LaserOdometryNode {
private:
    ros::NodeHandle handle;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    ros::Time lastScanTime;
    tf::TransformBroadcaster tfBroadcast;

    Scan *lastScan;
    ICP *icp;
    uint32_t counter;

    Eigen::Vector3d currentPose;
    Eigen::Matrix3d currentCovariance;

public:
    LaserOdometryNode();
    virtual ~LaserOdometryNode();
    void init();
    void run();
    void shutdown();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void setupICP();

    void sendTransform(const ros::Time &stamp);
    void sendOdometry(
            const ros::Time &stamp,
            const Eigen::Vector3d &displacement,
            const Eigen::Matrix3d &covariance);

};

#endif /* LASERODOMETRYNODE_H_ */
