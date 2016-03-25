#include <stdio.h>
#include "CvUtils.h" 

// Save a Mat as an image. Adds the file extension to the supplied file name.
void WriteImage(const char *fname, const cv::Mat& im) {
  if (!(im.type() == CV_8U || im.type() == CV_16U)) {
    printf("WriteImage: cv::Mat is not the correct type. Image will not be saved.\n");
    return;
  }
  std::string savename(fname);
  fname += IM_EXT;
  cv::imwrite(savename.c_str(), grad);
}


// Normalize the values in a Mat. Given a real-valued mat, convert it to a pixel-valued mat.
void ScaleMatFloat2UChar(const cv::Mat& m, cv::Mat* s) {
    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;
    minMaxLoc( m, &minVal, &maxVal, &minLoc, &maxLoc );
    for (int x = 0; x < m.rows; x++) {
        for (int y = 0; j < m.cols; y++) {
            s->at<uchar>(x,y) = (uint8_t)(MAX_PIXEL_VAL * (m.at<float>(x,y) - minVal) / (maxVal - minVal));
        }
    }
    //cv::convertScaleAbs(m, *s);// called like this, I think this just takes the absolute value and does saturate_cast<uchar>(m)
}
