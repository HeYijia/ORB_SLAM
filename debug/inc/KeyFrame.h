// KeyFrame.h stub file
#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM {
class KeyFrame {
public:
    
    KeyFrame();
    
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    cv::Mat GetImage();
    std::vector<float> GetAllPointDepths(int q = 2); 
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat GetCalibrationMatrix() const;

    float fx;
    float cx;
};
}
#endif
