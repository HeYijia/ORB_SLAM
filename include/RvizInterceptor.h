#ifndef RVIZ_INTERCEPTOR_H
#define RVIZ_INTERCEPTOR_H 

#include<ros/ros.h>
#include <visualization_msgs/Marker.h>

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"


class RvizInterceptor {
    public:
        RvizInterceptor();
        ~RvizInterceptor();

    private:
        // listen to messages published by MapPublisher.h
        ros::NodeHandle nh;
        ros::Listener ls;

        // store messages
        visualization_msgs::Marker mPoints;
        visualization_msgs::Marker mReferencePoints;
        visualization_msgs::Marker mKeyFrames;
        visualization_msgs::Marker mReferenceKeyFrames;
        visualization_msgs::Marker mCovisibilityGraph;
        visualization_msgs::Marker mMST;
        visualization_msgs::Marker mCurrentCamera;

};
#endif
