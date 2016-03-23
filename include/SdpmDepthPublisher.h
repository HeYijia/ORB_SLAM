/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SDPM_DEPTH_PUBLISHER_H
#define SDPM_DEPTH_PUBLISHER_H

#include<ros/ros.h>
#include <visualization_msgs/Marker.h>

#include"Map.h"
#include"MapPoint.h"
#include "ProbabilityMapping.h"

#include <vector>
#include<boost/thread.hpp>

namespace ORB_SLAM
{

class SdpmDepthPublisher
{
public:
    SdpmDepthPublisher();

    void Refresh();
    void SetPoints(const std::vector<cv::Mat>& new_points); 
    void PublishSdpmPoints();

private:
    ros::NodeHandle nh;
    ros::Publisher publisher;

    visualization_msgs::Marker mSdpmPoints;

    float fCameraSize;
    float fPointSize;

    std::vector<cv::Mat> points;
    bool updated;
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
