#include "RvizInterceptor.h"
#include <string.h>

RvizInterceptor::RvizInterceptor() {
    // configure node listener
    ls = nh.subscribe("ORB_SLAM/Map", 1, &RvizInterceptor::MapMsgCallback, this);
}

void RvizInterceptor::MapMsgCallback(const visualization_msgs::Marker &msg) {
/* namespaces used by MapPublisher:
    const char* POINTS_NAMESPACE = "MapPoints";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";
*/
    if (!strcmp(msg.ns, "/MapPoints")) {
        if (msg.id == MPOINTS_ID) {
            MapPoints = msg;
            ProcessMapPoints();
        } else if (msg.id == REF_ID) {
            ReferencePoints = msg;
            ProcessRefPoints();
        }
    } else if (!strcmp(msg.ns, "KeyFrames")) {
        KeyFrames = msg;
        ProcessKeyFrames();
    } else if (!strcmp(msg.ns, "Graph")) {
        if (msg.id == COVIS_ID) {
            CovisibilityGraph = msg;
            ProcessGraph();
        } else if (msg.id == MST_ID) {
            MST = msg;
            ProcessMST();
        }
    } else if (!strcmp(msg.ns, "Camera")) {
        CurrentCamera = msg;
        ProcessCamera();
    } else {
        ROS_INFO("RvizInterceptor: Unknown Message Type: %s\n", msg.ns.data.c_str());
    }
}
