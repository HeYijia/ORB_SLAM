#include <stdlib.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class Tests {
public:
    Tests() :test_image(cv::Mat::zeros(100,100, CV_8U)) {}
    Tests(const char* image_name) {
        SetTestImage(image_name); 
    }

    cv::Mat GetTestImage() { return test_image.clone(); }
    void SetTestImage(const char* image_name) {
        test_image = cv::imread( image_name );
    }

    // test methods
    int cv_scharr( const char* savename="output.jpg", bool use_scharr=true);

private:
    cv::Mat test_image;
};
