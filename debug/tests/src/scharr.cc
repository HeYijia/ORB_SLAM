#include <stdlib.h>
#include <stdio.h>

#include "Tests.h"
#include "CvUtils.h"

// computes the scharr (sobel) derivative of an image and saves the result to a file
int Tests::cv_scharr( const char* savename, bool use_scharr) {

  cv::Mat src, src_gray;
  cv::Mat grad;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  /// Load the test image
  src = GetTestImage();

  if( !src.data )
  { return EXIT_FAILURE; }

  cv::GaussianBlur( src, src, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

  /// Convert it to gray
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Generate grad_x and grad_y
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  /// Gradient X and Y
  if (use_scharr) {
      cv::Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT );
      cv::Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT );
  } else {
      cv::Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
      cv::Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
  }
  
  /// Total Gradient (approximate)
  cv::convertScaleAbs( grad_x, abs_grad_x );
  cv::convertScaleAbs( grad_y, abs_grad_y );
  cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  // write the image result
  WriteImage(savename, grad);

  return 0;
}
