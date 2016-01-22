#ifndef PROBABILITY_MAPPING_H
#define PROBABILITY_MAPPING_H

#define N 7
#define sigmaI 20
#define lambdaG 8
#define lambdaL 80
#define lambdaTheta 45
#define lambdaN 3

class ProbabilityMapping {

private:

    struct depth_ho {
        float depth; // pixel depth hypothesis
        float sigma; // pixel sigma depth hypothesis
        //int x; // pixel x
        //int y; // pixel y
    };

    void getImageGradient(cv::Mat& image, cv::Mat* grad);
    void getGradientOrientation(cv::Mat& grad, float th);
    void getInPlaneRotation(KeyFrame& k1, KeyFrame& k2);
    void getIntensityGradient(cv::Mat im, float* g);
    void getIntensityGradient_D(cv::Mat im, float* q);
    void getPixelDepth(cv::Mat& R,cv::Mat& T, cv::Mat& K, cv::Mat& Im, int x, int y, float *p);
    bool chiTest(float pa, float pb, float sa, float sb, float* chi_val);
    void depthDistribution(float p, float sigma, depth_ho* hypothesis);

public:

    // return results of epipolar search- depth hypothesis
    void first_loop(KeyFrame kf, depth_ho**, std::vector<depth_ho>&);
    // return min, max inverse depth
    void stereo_search_constraints();
    // return distribution of inverse depths/sigma (for each pixel)
    void epipolar_search();
    void inverse_depth_hypothesis_fusion();
    void intra_keyframe_depth_checking();
    void inter_keyframe_depth_checking();

};

#endif
