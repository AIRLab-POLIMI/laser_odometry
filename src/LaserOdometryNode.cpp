/*
 * LaserOdometryNode.cpp
 *
 *  Created on: 21/lug/2012
 *      Author: Mladen Mazuran
 */

#ifdef __DEBUG__
#   define DBG_PREFIX ""
#endif

#include "Config.h"
#include "ConfigLoader.h"
#include "ConfigPrinter.h"
#include "scanmatching/NearestNeighboursON2.h"
#include "scanmatching/NearestNeighboursANN.h"
#include "scanmatching/ClassicICP.h"
#include "scanmatching/MetricICP.h"
#include "scanmatching/PointToLineICP.h"
#include "Conversions.h"
#include "Equations.h"
#include "LaserOdometryNode.h"

#include <iostream>
#include <fstream>

LaserOdometryNode::LaserOdometryNode() :
    lastScan(NULL), icp(NULL), counter(0), currentPose(),
    currentCovariance(Eigen::Matrix3d::Zero())
{
}

LaserOdometryNode::~LaserOdometryNode()
{
    if(icp) {
        delete icp;
    }
}

void LaserOdometryNode::init(void)
{
    /* Initialise everything */

    loadConfig(handle);
    setupICP();

    subscriber = handle.subscribe(
            config::laser_topic, config::laser_queue_len, &LaserOdometryNode::laserCallback, this);
    publisher = handle.advertise<nav_msgs::Odometry>(
            config::odometry_topic, config::odometry_queue_len);

    ROS_INFO("Node %s ready to run", ros::this_node::getName().c_str());

    printConfig();
}

void LaserOdometryNode::run(void)
{
    ROS_INFO("Node %s running continuously", ros::this_node::getName().c_str());

    ros::spin();
}

void LaserOdometryNode::shutdown(void)
{
    ROS_INFO("Node %s shutting down", ros::this_node::getName().c_str());
}

void LaserOdometryNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    /* Time since last matched scan */
    const double dt = (scan->header.stamp - lastScanTime).toSec();

    //ROS_INFO("Laser Callback.");

    if(lastScan == NULL) {
        /* First scan */
        lastScan = conversions::rosToScan(scan);
        lastScanTime = scan->header.stamp;
    } else if(dt >= 1. / config::max_rate) {
        /* Runs with <= max_rate frequency */
        Eigen::Vector3d displacement;
        Eigen::Matrix3d covariance;
        Scan *newScan = conversions::rosToScan(scan);

        /* Run icp */
        icp->scanMatch(*lastScan, *newScan, displacement, covariance);

#ifdef __DEBUG__
        /* Debug stuff, output everything to a file */
        static std::ofstream *f = new std::ofstream(DBG_PREFIX "icp.txt", std::ios::out);
        *f << "last=" << *lastScan << ";" << std::endl;
        *f << "new=" << *newScan << ";" << std::endl;
        *f << "rotated=" << Rototranslation2D(displacement) * (*newScan) << ";" << std::endl;
        *f << "roto={" << displacement[0] << "," << displacement[1] << "," << displacement[2] << "};" << std::endl;
#endif

        Eigen::Vector3d newPose = equations::updatePose(currentPose, displacement);

        currentCovariance = equations::updatePoseCovariance(
                currentPose, currentCovariance, displacement, covariance);
        currentPose = newPose;

        if(config::send_transform) sendTransform(scan->header.stamp);
        sendOdometry(scan->header.stamp, displacement, covariance);

        delete lastScan;
        lastScan = newScan;
        lastScanTime = scan->header.stamp;
    }

}

void LaserOdometryNode::sendTransform(const ros::Time &stamp) {
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = config::tf_parent_frame;
    transform.child_frame_id = config::tf_child_frame;

    conversions::vectorToROSTransform(currentPose, transform.transform);

    tfBroadcast.sendTransform(transform);
}

void LaserOdometryNode::sendOdometry(
        const ros::Time &stamp,
        const Eigen::Vector3d &displacement,
        const Eigen::Matrix3d &covariance) {

    const double dt = (stamp - lastScanTime).toSec();
    nav_msgs::Odometry o;

    o.header.seq = counter++;
    o.header.stamp = stamp;

    o.header.frame_id = config::tf_parent_frame;
    o.child_frame_id = config::tf_child_frame;

    conversions::vectorToROSPose(currentPose, o.pose.pose);
    conversions::covarianceToROS6DOF(currentCovariance, o.pose.covariance);

#ifdef __DEBUG__
    static std::ofstream *f = new std::ofstream(DBG_PREFIX "roto.csv", std::ios::out);
    geometry_msgs::Point p = o.pose.pose.position;
    geometry_msgs::Quaternion q = o.pose.pose.orientation;
    const Eigen::Matrix3d &c = covariance;
    const char *s = " ";

    *f << stamp << s << displacement[0] << s << displacement[1] << s << displacement[2] <<
        s << c(0,0) << s << c(0,1) << s << c(0,2) <<
        s << c(1,0) << s << c(1,1) << s << c(1,2) <<
        s << c(2,0) << s << c(2,1) << s << c(2,2) <<
        std::endl;

#endif

    /* TODO: These are WRONG! Transformation != derivative */
    conversions::twistToROS6DOF(displacement / dt, o.twist.twist);
    conversions::twistCovarianceToROS6DOF(covariance / dt / dt, o.twist.covariance);

    publisher.publish(o);
}

void LaserOdometryNode::setupICP()
{
    NearestNeighbours *nn = NULL;

    switch(config::nn_engine) {
    case config::ON2:
        nn = new NearestNeighboursON2();
        break;
    case config::ANN:
        nn = new NearestNeighboursANN();
        break;
    }

    switch(config::icp_algorithm) {
    case config::Classic:
        icp = new ClassicICP(nn);
        break;
    case config::Metric:
        icp = new MetricICP(config::icp_metric_l);
        break;
    case config::PointToLine:
        icp = new PointToLineICP(nn);
        break;
    }
}
