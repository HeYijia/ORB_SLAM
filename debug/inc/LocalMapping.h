
#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include <opencv2/opencv.hpp>

namespace ORB_SLAM {

class KeyFrame;

class LocalMapping {
    public:
    static cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
};

}

#endif
