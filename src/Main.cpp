/*
 * Main.cpp
 *
 *  Created on: 11/nov/2012
 *      Author: Mladen Mazuran
 */

#include "LaserOdometryNode.h"
#include "Config.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, config::node_name);

    LaserOdometryNode node;

    node.init();
    node.run();
    node.shutdown();

    return 0;
}
