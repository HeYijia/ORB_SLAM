#include <stdlib.h>
#include <opencv2/opencv.hpp>

#define MAX_PIXEL_VALUE 255
#define MIN_PIXEL_VAL 0
#define WHITE 255
#define BLACK 0

#define IM_EXT ".jpg"

void WriteImage(const char *fname, const cv::Mat& im);
void ScaleMatFloat2UChar(const cv::Mat& m, cv::Mat* s);
