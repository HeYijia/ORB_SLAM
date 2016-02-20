#include "KeyFrame.h"

namespace ORB_SLAM {

KeyFrame::KeyFrame() {}

cv::Mat KeyFrame::GetImage() {
    cv::Mat im;
    return im;
}

cv::Mat KeyFrame::GetRotation() {
    return GetImage();
}

cv::Mat KeyFrame::GetTranslation() {
    return GetImage();
}

std::vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
    std::vector<KeyFrame*> v;
    return v;
}

std::vector<float> KeyFrame::GetAllPointDepths(int q) {
    std::vector<float> v;
    return v;
}

cv::Mat KeyFrame::GetCalibrationMatrix() const {
    cv::Mat mK;
    return mK.clone();
}

}
