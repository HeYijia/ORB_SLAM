#ifndef PROBABILITY_MAPPING_H
#define PROBABILITY_MAPPING_H

#define N 7//unused in pseudocode ...?
#define covisN 20
#define sigmaI 20
#define lambdaG 8
#define lambdaL 80
#define lambdaTheta 45
#define lambdaN 3

class ORB_SLAM::KeyFrame;
class cv::Mat;

class ProbabilityMapping {

private:

    struct depthHo {
        float depth; // pixel depth hypothesis
        float sigma; // pixel sigma depth hypothesis
    };

    /** \brief void getImageGradient(cv::Mat& image, cv::Mat* grad): calculate pixel gradient over entire image */
    void getImageGradient(cv::Mat& image, cv::Mat* grad);
    /** \brief void getGradientOrientation(cv::Mat& grad, float th): calculate gradient orientation over entire image */
    void getGradientOrientation(cv::Mat& grad, float th); 
    /** \brief void getInPlaneRotation(ORB_SLAM::KeyFrame& k1, ORB_SLAM::KeyFrame& k2): get in-plane rotation between two keyframes */
    void getInPlaneRotation(const ORB_SLAM::KeyFrame* k1, const ORB_SLAM::KeyFrame* k2);
    /** \brief void getIntensityGradient(cv::Mat im, float* g): calculate intensity gradient of an image (Eq 5) */
    void getIntensityGradient(cv::Mat im, int x, int y, float* g);
    /** \brief void getIntensityGradient_D(cv::Mat im, float* q): calculate the derivate of the intensity gradient of an image (Eq 5) */
    void getIntensityGradient_D(cv::Mat im, float* q); 
    /** \brief void getPixelDepth(cv::Mat& R,cv::Mat& T, cv::Mat& K, cv::Mat& Im, int x, int y, float *p): calculate pixel inverse depth 
        R: rotation matrix; T: translation matrix; K: keyframe; Im: the image (pixel matrix) (Eq 8) */
    void getPixelDepth(cv::Mat& R, cv::Mat& T, ORB_SLAM::KeyFrame* K, cv::Mat& Im, int x, int y, float *p);
    /** \brief bool chiTest(const depthHo ha, const depthHo hb, float* chi_val): test whether two hypotheses are compatible (Eq 10) */
    bool chiTest(const depthHo ha, const depthHo hb, float* chi_val);
    /** \brief void depthDistribution(float p, float sigma, depthHo*
     * hypothesis): Given a set of compatible depth hypotheses, compute the
     * fused hypothesis, yielding the parameters for inverse depth distribution
     * for a pixel (Eq 11). Also return the sigma representing the lowest
     * standard deviation */
    void getFusion(const std::vector<depthHo> best_compatible_ho, depthHo* hypothesis, float* min_sigma);
    // should also have something for pseudocode.cc ~line 120
public:

    /** \brief void first_loop(ORB_SLAM::KeyFrame kf, depthHo**, std::vector<depthHo>&): return results of epipolar search (depth hypotheses) */
    void firstLoop(const ORB_SLAM::KeyFrame* Kf, depthHo**, std::vector<depthHo>&);
    /** \brief void stereo_search_constraints(): return min, max inverse depth */
    void stereoSearch_constraints();
    /** \brief void epipolar_search(): return distribution of inverse depths/sigmas for each pixel */
    void epipolarSearch();
    /** \brief void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist): 
        get the parameters of depth hypothesis distrubution from list of depth hypotheses */
    void inverseDepthHypothesisFusion(const std::vector<depthHo> H, depthHo* dist);
    /** \brief void intraKeyframeDepthChecking(depthHo** H, int imrows, int imcols): intra-keyframe depth-checking, smoothing, and growing. 
        Modifies the double-array of depth hypotheses provided as input */
    void intraKeyframeDepthChecking(depthHo** H, int imrows, int imcols);
    /** \brief void interKeyframeDepthChecking(const ORB_SLAM::KeyFrame* currentKF, std::vector<ORB_SLAM::KeyFrame*> neighbors, 
                                               depthHo** H, int imrows, int imcols): 
        Inter-keyframe depth-checking, smoothing, and growing. 
        Modifies the double-array of depth hypotheses provided as input */
    void interKeyframeDepthChecking(const ORB_SLAM::KeyFrame* currentKF, std::vector<ORB_SLAM::KeyFrame*> neighbors, depthHo** H, int imrows, int imcols);

};

#endif
