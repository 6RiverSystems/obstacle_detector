/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include <srslib_framework/MsgTrackingObstacles.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include "obstacle_detector/obstacle_adapter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;
using namespace obstacle_detector;

ObstacleAdapter::ObstacleAdapter(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  obstacles_sub_ = nh_.subscribe("obstacles", 10, &ObstacleAdapter::publishObstaclesCallback, this);
  obstacles_pub_ = nh_.advertise<costmap_converter::ObstacleArrayMsg>("tracked_obstacles", 10);
}

ObstacleAdapter::~ObstacleAdapter() {

}

void ObstacleAdapter::publishObstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles) {
    costmap_converter::ObstacleArrayMsgPtr obstacles_msg(new costmap_converter::ObstacleArrayMsg);

    obstacles_msg->header = obstacles->header;
     for(size_t i = 0; i < obstacles->circles.size(); i++) {
         costmap_converter::ObstacleMsg circularObstacle;
         circularObstacle.id = i;
         circularObstacle.radius = obstacles->circles[i].true_radius;
         geometry_msgs::Point32 center;
         center.x = obstacles->circles[i].center.x;
         center.y = obstacles->circles[i].center.y;
         center.z = obstacles->circles[i].center.z;
         circularObstacle.polygon.points.push_back(center);

         float yaw = std::atan2(obstacles->circles[i].velocity.y, obstacles->circles[i].velocity.x);
         tf2::Quaternion q;
         q.setRPY(0,0,yaw);
         circularObstacle.orientation.x = q.getX();
         circularObstacle.orientation.y = q.getY();
         circularObstacle.orientation.z = q.getZ();
         circularObstacle.orientation.w = q.getW();

         circularObstacle.velocities.twist.linear.x = obstacles->circles[i].velocity.x;
         circularObstacle.velocities.twist.linear.y = obstacles->circles[i].velocity.y;
         circularObstacle.velocities.twist.linear.z = 0;
         circularObstacle.velocities.twist.angular.x = 0;
         circularObstacle.velocities.twist.angular.y = 0;
         circularObstacle.velocities.twist.angular.z = 0;

        obstacles_msg->obstacles.push_back(circularObstacle);
  }

  obstacles_pub_.publish(obstacles_msg);
}
