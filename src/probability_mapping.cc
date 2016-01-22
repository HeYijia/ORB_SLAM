#include "probability_mapping.h"
#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include <vector>

/////////////////////
// process methods //
/////////////////////

// INVERSE DEPTH HYPOTHESIS FUSION 
// check compatibility of each hypothesis with each of the others
// Fuse groups of compatible hypotheses which are at least lambda_N in size
// chi test (compatibility) should be a function 
// inputs: an array of depth hypotheses
// outputs: a hypothesis distribution
void inverse_depth_hypothesis_fusion(const vector<depth_ho> H, depth_ho* dist) {
    // "clear" the distribution parameters
    dist...depth = 0;
    dist...sigma = 0;

    vector<depth_ho> best_compatible_ho;
    
    for (int a=0; a < H.size(); a++) {
        vector<depth_ho> compatible_ho;
        
        for (int b=0; b < H.size(); b++) {
            
            float pa = H[a].depth;
            float pb = H[b].depth;
            
            float chi_test = (pa - pb)*(pa - pb) / (H[a].sigma*H[a].sigma) + (pa - pb)*(pa - pb) / (H[a].sigma*H[a].sigma);
            
            // test if the hypotheses a and b are compatible
            if (chi_test(H[a], H[b], NULL)) {
                compatible_ho.push_back(H[b]); 
            }
        }
        // test if hypothesis 'a' has the required support
        if (compatible.size()-1 >= lambda_N && compatible.size() > best_compatible_depth.size()) {
            compatible_ho.push_back(H[a]); 
            best_compatible_ho = compatible_ho;
        }
    }

    // calculate the parameters of the normal distribution by fusing hypotheses
    float ndepthp = -1;
    float nsigma_sqr_p = -1;
    if (best_compatible_ho.size() >= lambda_N) {
        // make this a method?
        float pjsj =0; // numerator
        float rsj =0; // denominator
        for (int j = 0; j < best_compatible_ho.size(); j++) {
            pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
            rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
        }
        ndepthp = pjsj / rsj;
        nsigma_sqr_p = 1 / rsj;
    }


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

bool chiTest(const depth_ho ha, const depth_ho hb, float* chi_val) {
    float chi_test = (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma) + (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma);
    if (chi_val)
        *chi_val = chi_test;
    return (chi_test < 5.99);
} 

void depthDistribution(float p, float sigma, depth_ho* hypothesis) {

} 

