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

#include "SdpmDepthPublisher.h"
#include "MapPoint.h"
#include "ProbabilityMapping.h"

namespace ORB_SLAM
{

SdpmDepthPublisher::SdpmDepthPublisher()
{
    updated=false;

    const char* MAP_FRAME_ID = "/ORB_SLAM/SDPM_MapFrameId";
    const char* POINTS_NAMESPACE = "SDPM_PointsNamespace";

    //Configure SDPM Depth Map MapPoints
    fPointSize=0.01;
    mSdpmPoints.header.frame_id = MAP_FRAME_ID;
    mSdpmPoints.ns = POINTS_NAMESPACE;
    mSdpmPoints.id=0;
    mSdpmPoints.type = visualization_msgs::Marker::POINTS;
    mSdpmPoints.action=visualization_msgs::Marker::ADD;
    mSdpmPoints.scale.x=fPointSize;
    mSdpmPoints.scale.y=fPointSize;
    mSdpmPoints.pose.orientation.w=1.0;
    mSdpmPoints.color.b = 0.7f;
    mSdpmPoints.color.r = 0.7f;
    mSdpmPoints.color.a = 1.0;

    //Configure Publisher
    publisher = nh.advertise<visualization_msgs::Marker>("ORB_SLAM/SDPMap", 10);
    publisher.publish(mSdpmPoints);
}

void SdpmDepthPublisher::SetPoints(const std::vector<cv::Mat>& new_points) {
    points.assign(new_points.begin(), new_points.end()); //should probably be a mutex here
    updated=true;
}

void SdpmDepthPublisher::Refresh() {
    if (updated) {
        PublishSdpmPoints();
    }
}

void SdpmDepthPublisher::PublishSdpmPoints()
{
    mSdpmPoints.points.clear();
    // semi dense map points
    for(size_t i=0, iend=points.size(); i<iend;i++)
    {
        geometry_msgs::Point p;
        p.x=points[i].at<float>(0);
        p.y=points[i].at<float>(1);
        p.z=points[i].at<float>(2);

        mSdpmPoints.points.push_back(p);
    }

    mSdpmPoints.header.stamp = ros::Time::now();
    publisher.publish(mSdpmPoints);
    updated=false;
}

} //namespace ORB_SLAM
