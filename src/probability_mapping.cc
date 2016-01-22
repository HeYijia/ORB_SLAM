#include "probability_mapping.h"
#include "KeyFrame.h"
#include <opencv2/core/core.hpp>

/////////////////////
// process methods //
/////////////////////

void inverse_depth_hypothesis_fusion() {

} 

void intra_keyframe_depth_checking() {

} 

void inter_keyframe_depth_checking() {

} 

/////////////////////
// utility methods //
/////////////////////

void getIntensityGradient_D(cv::Mat im, float* q) {

} 

void getPixelDepth(cv::Mat& R,cv::Mat& T, cv::Mat& K, cv::Mat& Im, int x, int y, float *p) {

} 

bool chiTest(float pa, float pb, float sa, float sb, float* chi_val) {

} 

void depthDistribution(float p, float sigma, depth_ho* hypothesis) {

} 

