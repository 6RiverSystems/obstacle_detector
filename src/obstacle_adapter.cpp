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
#include "obstacle_detector/obstacle_adapter.h"

using namespace std;
using namespace obstacle_detector;

ObstacleAdapter::ObstacleAdapter(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  obstacles_sub_ = nh_.subscribe("obstacles", 10, &ObstacleAdapter::publishObstaclesCallback, this);
  obstacles_pub_ = nh_.advertise<srslib_framework::MsgTrackingObstacles>("tracked_obstacles", 10);
}

ObstacleAdapter::~ObstacleAdapter() {

}

void ObstacleAdapter::publishObstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles) {
  srslib_framework::MsgTrackingObstaclesPtr obstacles_msg(new srslib_framework::MsgTrackingObstacles);

  obstacles_msg->header = obstacles->header;
  for(size_t i = 0; i < obstacles->circles.size(); i++) {
    srslib_framework::CircularObstacle circularObstacle;
    circularObstacle.center = obstacles->circles[i].center;
    circularObstacle.velocity = obstacles->circles[i].velocity;
    circularObstacle.radius = obstacles->circles[i].radius;

    obstacles_msg->trackingObstacles.push_back(circularObstacle);
  }

  obstacles_pub_.publish(obstacles_msg);
}
