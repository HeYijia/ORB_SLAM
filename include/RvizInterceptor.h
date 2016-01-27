#ifndef RVIZ_INTERCEPTOR_H
#define RVIZ_INTERCEPTOR_H 

#include<ros/ros.h>
#include <visualization_msgs/Marker.h>

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"

// map points
#define MPOINTS_ID 0
// keyframes
#define KFRAMES_ID 1
// covisibility graph
#define COVIS_ID 2
// keyframes spanning tree
#define MST_ID 3
// current camera
#define CAM_ID 4
// reference map points
#define REF_ID 5

class RvizInterceptor {
    public:
        RvizInterceptor();

    private:
        // listen to messages published by MapPublisher.h
        ros::NodeHandle nh;
        ros::Subscriber ls; 
        // callback methods
        void MapMsgCallback(const visualization_msgs::Marker &msg);
        // processing methods
        void ProcessMapPoints();
        void ProcessKeyFrames();
        void ProcessGraph();
        void ProcessMST();
        void ProcessCamera();
        void ProcessRefPoints();
        // message holder variables
        visualization_msgs::Marker MapPoints;
        visualization_msgs::Marker ReferencePoints;
        visualization_msgs::Marker KeyFrames;
        visualization_msgs::Marker CovisibilityGraph;
        visualization_msgs::Marker MST;
        visualization_msgs::Marker CurrentCamera;

};
#endif
