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

    struct depthHo {
        float depth; // pixel depth hypothesis
        float sigma; // pixel sigma depth hypothesis
        //int x; // pixel x
        //int y; // pixel y
    };

    /** \brief void getImageGradient(cv::Mat& image, cv::Mat* grad): calculate pixel gradient over entire image */
    void getImageGradient(cv::Mat& image, cv::Mat* grad);
    /** \brief void getGradientOrientation(cv::Mat& grad, float th): calculate gradient orientation over entire image */
    void getGradientOrientation(cv::Mat& grad, float th); 
    /** \brief void getInPlaneRotation(KeyFrame& k1, KeyFrame& k2): get in-plane rotation between two KeyFrames */
    void getInPlaneRotation(KeyFrame& k1, KeyFrame& k2);
    /** \brief void getIntensityGradient(cv::Mat im, float* g): calculate intensity gradient of an image (Eq 5) */
    void getIntensityGradient(cv::Mat im, int x, int y, float* g);
    /** \brief void getIntensityGradient_D(cv::Mat im, float* q): calculate the derivate of the intensity gradient of an image (Eq 5) */
    void getIntensityGradient_D(cv::Mat im, float* q); 
    /** \brief void getPixelDepth(cv::Mat& R,cv::Mat& T, cv::Mat& K, cv::Mat& Im, int x, int y, float *p): calculate pixel inverse depth 
        R: rotation matrix; T: translation matrix; K: keyframe; Im: the image (pixel matrix) (Eq 8) */
    void getPixelDepth(cv::Mat& R,cv::Mat& T, KeyFrame* K, cv::Mat& Im, int x, int y, float *p);
    /** \brief bool chiTest(const depthHo ha, const depthHo hb, float* chi_val): test whether two hypotheses are compatible (Eq 10) */
    bool chiTest(const depthHo ha, const depthHo hb, float* chi_val);
    /** \brief void depthDistribution(float p, float sigma, depthHo* hypothesis): calculate the parameters for inverse depth distribution for a pixel (Eq 11) */
    void depthDistribution(const vector<depthHo> best_compatible_ho, depthHo* hypothesis);
    // should also have something for pseudocode.cc ~line 120
public:

    /** \brief void first_loop(KeyFrame kf, depthHo**, std::vector<depthHo>&): return results of epipolar search (depth hypotheses) */
    void firstLoop(KeyFrame kf, depthHo**, std::vector<depthHo>&);
    /** \brief void stereo_search_constraints(): return min, max inverse depth */
    void stereoSearch_constraints();
    /** \brief void epipolar_search(): return distribution of inverse depths/sigmas for each pixel */
    void epipolarSearch();
    /** \brief void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist): 
        get the parameters of depth hypothesis distrubution from list of depth hypotheses */
    void inverseDepthHypothesisFusion(const vector<depthHo> H, depthHo* dist);
    /** \brief void intraKeyframeDepthChecking(depthHo** H): intra-keyframe depth-checking, smoothing, and growing. 
        Modifies the double-array of depth hypotheses provided as input */
    void intraKeyframeDepthChecking(depthHo** H);
    /** \brief void interKeyframeDepthChecking(depthHo** H): inter-keyframe depth-checking, smoothing, and growing. 
        Modifies the double-array of depth hypotheses provided as input */
    void interKeyframeDepthChecking(depthHo** H);

};

#endif
