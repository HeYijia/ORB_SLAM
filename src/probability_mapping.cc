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
// outputs: an inverse depth distribution
void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist) {
    // "clear" the distribution parameters
    dist...depth = 0;
    dist...sigma = 0;

    vector<depthHo> best_compatible_ho;
    
    for (int a=0; a < H.size(); a++) {
        vector<depthHo> compatible_ho;
        
        for (int b=0; b < H.size(); b++) {
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

    // calculate the parameters of the inverse depth distribution by fusing hypotheses
    if (best_compatible_ho.size() >= lambda_N) {
        depthDistribution(best_compatible_ho, dist);
    }
} 

// INTRA - KEYFRAME DEPTH CHECKING, SMOOTHING, AND GROWING
void intra_keyframe_depth_checking(depthHo** H) {

    // run through all the pixels again
    depthHo depthsArr2[original.rows][original.cols];
    for (int px = 1; px < (original.rows - 1); px++) {
        for (int py = 1; py < (original.cols - 1); py++) {
            depthsArr2[px][py] = NULL;
            if (depthsArr[px][py] == NULL) {
                // check if this pixel is surrounded by at least two pixels that are compatible to each other.
                std::vector<std::vector<depthHo>> best_compatible_ho;
                std::vector<float> min_sigmas;
                for (int nx = px - 1; nx <= px + 1; nx++) {
                    for (int ny = py - 1; ny <= py + 1; ny++) {
                        if (nx == px && ny == py) continue;
                        std::vector<depthHo> temp;
                        // go through the neighbors again!
                        float min_sigma = 100;
                        for (int nnx = px - 1; nnx <= px + 1; nnx++) {
                            for (int nny = py - 1; nny <= py + 1; nny++) {
                                if ((nnx == nx && nny == ny) || (nnx == px && nny == py)) continue;
                                float pa = depthsArr[nx][ny].depth; 
                                float pb = depthsArr[nnx][nny].depth; 
                                float chi_test = (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma) + (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma);
                                if (chi_test < 5.99) {
                                    temp.push_back(depthsArr[nnx][nny]);
                                    if (depthsArr[nnx][nny].sigma * depthsArr[nnx][nny].sigma < min_sigma * min_sigma) {
                                        min_sigma = depthsArr[nnx][nny].sigma;
                                    }
                                }
                            }
                        }
                        min_sigmas.push_back(min_sigma); 
                        best_compatible_ho.push_back(temp);
                    }
                }
                // potentially grow the reconstruction density
                int max_support = 0;
                int max_support_index = 0;
                for (int c = 0; c < best_compatible_ho.size(); c++) {
                    if (best_compatible_ho[c].size() > max_support) {
                        max_support = best_compatible_ho[c].size();
                        max_support_index = c;
                    }
                }
                if (max_support >= 2) {
                    // assign this previous NULL depthHo the average depth and min sigma of its compatible neighbors
                    float avg = 0;

                    float pjsj =0; // numerator
                    float rsj =0; // denominator
                    for (int j = 0; j < best_compatible_ho[max_support_index].size(); j++) {
                        pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                        rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                    }
                    avg = pjsj / rsj;

                    depthsArr2[px][py].depth = avg;
                    depthsArr2[px][py].sigma = min_sigma;
                }
            } else {
                // calculate the support of the pixel's  8 neighbors
                int support_count = 0;
                float chi_test, pa, pb;
                float min_sigma = 100;
                std::vector<depthHo> best_compatible_ho;
                for (int nx = px - 1; nx <= px + 1; nx++) {
                    for (int ny = py - 1; ny <= ny + 1; ny++) {
                        if (nx == px && ny == py) continue;
                        if (depthsArr[nx][ny] == NULL) continue;
                        pa = depthsArr[px][py].depth;
                        pb = depthsArr[nx][ny].depth;
                        chi_test = (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma) + (pa - pb)*(pa - pb) / (depths[a].sigma*depths[a].sigma);
                        if (chi_test < 5.99) {
                            support_count++;
                            best_compatible_ho.push_back(depthsArr[nx][ny]);
                            if (depthsArr[nx][ny].sigma * depthsArr[nx][ny].sigma < min_sigma * min_sigma) {
                                min_sigma = depthsArr[nx][ny].sigma;
                            }
                        }
                    }
                }
                if (support_count < 2) {
                    // average depth of the retained pixels
                    // set stdev to minimum of neighbor pixels
                    float avg = 0;

                    float pjsj =0; // numerator
                    float rsj =0; // denominator
                    for (int j = 0; j < best_compatible_ho.size(); j++) {
                        pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                        rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
                    }
                    avg = pjsj / rsj;

                    depthsArr2[px][py].depth = avg;
                    depthsArr2[px][py].sigma = min_sigma;
                } else {
                    depthsArr2[px][py] = depthsArr[px][py];
                }
            }
        }
    }


} 

void inter_keyframe_depth_checking() {

} 

/////////////////////
// utility methods //
/////////////////////

void getIntensityGradient_D(cv::Mat ImGrad, float* q) {
    float grad_d = (ImGrad.at<float>(uplusone,vplusone) - ImGrad.at<float>(uminone,vminone))/2;
    *q = grad_d;
} 

void getPixelDepth(cv::Mat& R, cv::Mat& T, KeyFrame* kF cv::Mat& Im, int u, float *p) {
    const float fx = kF->fx;
    const float cx = kF->cx;

    cv::Mat K = kF->GetCalibrationMatrix();

    cv::Mat Xp = K*Im;

    int ucx = u - cx;
    int vcx = (a/b)*ucx + (c/b);
    
    float depthp = (R[2]*Xp.at<float>(ucx,vcx)-fx*R[0]*Xp)/(-T[2][ucx][vcx]+fx*T[0]);
    *p = depthp;
} 

bool chiTest(const depthHo ha, const depthHo hb, float* chi_val) {
    float chi_test = (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma) + (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma);
    if (chi_val)
        *chi_val = chi_test;
    return (chi_test < 5.99);
} 

void depthDistribution(const vector<depthHo> best_compatible_ho, depthHo* hypothesis) {
    hypothesis...depth = 0;
    hypothesis...sigma = 0;
    float pjsj =0; // numerator
    float rsj =0; // denominator
    for (int j = 0; j < best_compatible_ho.size(); j++) {
        pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
        rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
    }
    hypothesis...depth = pjsj / rsj;
    hypothesis...sigma = sqrt(1 / rsj);
} 

