/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.h
 *
 *    Description:  Semi-dense Probability Mapping Module for ORB-SLAM
 *    inspired by Raul-Mur Artal's paper
 *
 *        Version:  0.01
 *        Created:  01/21/2016 03:48:26 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Authors: Josh Tang, Rebecca Frederick 
 *   Organization:  Unkei
 *
 * =====================================================================================
 */

#ifndef PROBABILITYMAPPING_H
#define PROBABILITYMAPPING_H

#define N 7
#define sigmaI 20
#define lambdaG 8
#define lambdaL 80
#define lambdaTheta 45
#define lambdaN 3
#define histo_length 30
#define th_high = 100
#define th_low = 50
#define nnratio 0.6

class ProbabilityMapping {

private:

	struct depthHo {
		float depth;
		float sigma;
	};
	
	void getImageGradient(cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* grad);
    	void getGradientOrientation(cv::Mat& gradx, cv::Mat& grady, float th);
        void getInPlaneRotation(ORB_SLAM::KeyFrame& k1, ORB_SLAM::KeyFrame& k2);
	void getIntensityGradient(cv::Mat im, float* g);
        void getIntensityGradient_D(cv::Mat im, float* q);				
        void getPixelDepth(cv::Mat& R, cv::Mat& T, ORB_SLAM::KeyFrame* K, cv::Mat& Im, int x, int y, float *p);
	bool chiTest(const depthHo ha, const depthHo hb, float* chi_val);
	void getFusion(const std::vector<depthHo> best_compatible_ho, depthHo* hypothesis, float* min_sigma);

public:

	/* * \brief void firstLoop(ORB_SLAM::KeyFrame kf, depthHo**, std::vector<depthHo>&): return results of epipolar search (depth hypotheses) */
	void firstLoop(ORB_SLAM::KeyFrame kf, depthHo**, std::vector<depthHo>&);
        /* * \brief void stereoSearchConstraints(KeyFrame kf, float* min_depth, float* max_depth): return min, max inverse depth */
        void stereoSearchConstraints(KeyFrame kf, float* min_depth, float* max_depth){
	/* * \brief void epipolarSearch(KeyFrame kf1, Keyframe kf2, int x, int y, cv::Mat gradx, cv::Mat grady, cv::Mat grad, float min_depth, float max_depth, depthHo* dh): return distribution of inverse depths/sigmas for each pixel */
        void epipolarSearch(KeyFrame kf1, Keyframe kf2, int x, int y, cv::Mat gradx, cv::Mat grady, cv::Mat grad, float min_depth, float max_depth, depthHo* dh){
	/* * \brief void inverse_depth_hypothesis_fusion(const std::vector<depthHo>& H, depthHo* dist): 
	 * *         get the parameters of depth hypothesis distrubution from list of depth hypotheses */
        void inverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo* dist);
	/* * \brief void intraKeyframeDepthChecking(depthHo** h, int imrows, int imcols): intra-keyframe depth-checking, smoothing, and growing. */
        void intraKeyframeDepthChecking(depthHo** h, int imrows, int imcols);
	/* * \brief void interKeyframeDepthChecking(const ORB_SLAM::KeyFrame* currentKF, depthHo** h, int imrows, int imcols): 
         * *         inter-keyframe depth-checking, smoothing, and growing. */
        void interKeyframeDepthChecking(const ORB_SLAM::KeyFrame* currentKF, depthHo** h, int imrows, int imcols);

};

#endif
