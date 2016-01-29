#include "probability_mapping.h"
#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include <vector>

/////////////////////
// process methods //
/////////////////////

// todo:
// mark where equations from SDPM paper appear in code


// INVERSE DEPTH HYPOTHESIS FUSION (per pixel):
// check compatibility of each hypothesis with each of the others
// Fuse groups of compatible hypotheses which are at least lambdaN in size
// chi test (compatibility) should be a function 
// inputs: an array of depth hypotheses
// outputs: an inverse depth distribution
void inverse_depth_hypothesis_fusion(const vector<depthHo> H, depthHo* dist) {
    dist->depth = 0;
    dist->sigma = 0;

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
        if (compatible.size()-1 >= lambdaN && compatible.size() > best_compatible_depth.size()) {
            compatible_ho.push_back(H[a]); 
            best_compatible_ho = compatible_ho;
        }
    }

    // calculate the parameters of the inverse depth distribution by fusing hypotheses
    if (best_compatible_ho.size() >= lambdaN) {
        depthDistribution(best_compatible_ho, dist, NULL);
    }
} 

void pixelNeighborSupport (const depthHo** H, int px, int py, std::vector<depthHo>& support) {
    support.clear();
    for (int x = px - 1; x <= px + 1; x++) {
        for (int y = py - 1; y <= py + 1; y++) {
            if (x == px && y == py) continue; 
            if (chi_test(H[x][y], H[px][py], NULL)) {
                support.push_back(H[px][py]);
            }
        }
    }
}

void pixelNeighborNeighborSupport (const depthHo** H, int px, int py, std::vector<std::vector<depthHo>& support) {
    support.clear();
    for (int x = px - 1; x <= px + 1; x++) {
        for (int y = py - 1; y <= py + 1; y++) {
            if (x == px && y == py) continue;
            std::vector<depthHo> tempSupport;
            float min_sigma = H[x][y].sigma;
            for (int nx = px - 1; nx <= px + 1; nx++) {
                for (int ny = py - 1; ny <= py + 1; ny++) {
                    if ((nx == px && ny == py) || (nx == x && ny == y)) continue;
                    if (chi_test(H[x][y], H[nx][ny], NULL) {
                        tempSupport.push_back(H[nx][ny]);
                    }
                }
            }
            support.push_back(tempSupport);
        }
    }
}

// INTRA-KEYFRAME DEPTH CHECKING, SMOOTHING, AND GROWING
void intra_keyframe_depth_checking(depthHo** H, int imrows, int imcols) {

    depthHo H_new[imrows][imcols];
    for (int px = 1; px < (imrows - 1); px++) {
        for (int py = 1; py < (imcols - 1); py++) {
            H_new[px][py] = NULL;
            if (H[px][py] == NULL) {
                // check if this pixel is surrounded by at least two pixels that are compatible to each other.
                std::vector<std::vector<depthHo>> best_compatible_ho;
                
                pixelNeighborNeighborSupport (H, px, py, best_compatible_ho);
                
                int max_support = 0;
                int max_support_index = 0;
                for (int c = 0; c < best_compatible_ho.size(); c++) {
                    if (best_compatible_ho[c].size() > max_support) {
                        max_support = best_compatible_ho[c].size();
                        max_support_index = c;
                    }
                }
                
                // potentially grow the reconstruction density
                if (max_support >= 2) {
                    // assign this previous NULL depthHo the average depth and min sigma of its compatible neighbors
                    depthHo fusion;
                    float min_sigma;

                    getFusion(best_compatible_ho[c], &fusion, &min_sigma);

                    H_new[px][py].depth = fusion.depth;
                    H_new[px][py].sigma = min_sigma;
                }

            } else {
                // calculate the support of the pixel's  8 neighbors
                std::vector<depthHo> best_compatible_ho;
                
                pixelNeighborSupport(H, px, py, best_compatible_ho);
                
                if (best_compatible_ho.size() < 2) {
                    // average depth of the retained pixels
                    // set sigma to minimum of neighbor pixels
                    depthHo fusion;
                    float min_sigma;

                    getFusion(best_compatible_ho, &fusion, &min_sigma);

                    H_new[px][py].depth = fusion.depth;
                    H_new[px][py].sigma = min_sigma;

                } else {
                    H_new[px][py] = H[px][py];
                }
            }
        }
    }
    
    for (xnt x = 0; x < imrows; x++) {
        for (int y = 0; y < imcols; y++) {
            H[x][y] = H_new[x][y];
        }
    }
} 

void getNeighborKeyFrames (const ORB_SLAM::KeyFrame* Kf, std::vector<ORB_SLAM::KeyFrame*>& neighbors) {
    neighbors.clear();
    // option1: could just be the best covisibility keyframes
    neighbors = Kf->GetBestCovisibilityKeyFrames(covisN);
    // option2: could be found in one of the LocalMapping SearchByXXX() methods
    ORB_SLAM::LocalMapping::SearchInNeighbors(); //mpCurrentKeyFrame->updateConnections()...AddConnection()...UpdateBestCovisibles()...
    ORB_SLAM::LocalMapping::mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(covisN); //mvpOrderedConnectedKeyFrames()
}

// INTER-KEYFRAME DEPTH CHECKING AND SMOOTHING
void inter_keyframe_depth_checking(const ORB_SLAM::KeyFrame* currentKF, std::vector<ORB_SLAM::KeyFrame*> neighbors, depthHo** H, int imrows, int imcols) {
        
        // for each pixel of keyframe_i, project it onto each neighbor keyframe keyframe_j
        // and propagate inverse depth
        for (int px = 0; px < imrows; px++) {
            for (int py = 0; py < imcols; py++) {
                if (H[px][py] == NULL) continue; 
                float depthp = H[px][py].depth;
                int compatible_neighbor_keyframes_count = 0; // count of neighboring keyframes in which there is at least one compatible pixel
	        for(int j=0; j<neighbors.size(); j++){ 
		    ORB_SLAM::KeyFrame* pKFj = neighbors[j];
                    // calibration matrix
	            cv::Mat Kj = pKFj -> GetCalibrationMatrix;
		    cv::Mat Xp = Kj*image;
                    // rotation matrix
                    cv::Mat Rcwj = Tcw_.row(2).colRange(0,3);
                    Rcwj = Rcwj.t();
                    // translation matrix
                    cv::Mat tcwj = pKF2->GetTranslation();
                    cv::Mat Tcwj(3,4,CV_32F);
                    Rcwj.copyTo(Tcwj.colRange(0,3));
                    tcwj.copyTo(Tcwj.col(3));
                    
                    // compute the projection matrix to map 3D point from original image to 2D point in neighbor keyframe
                    // Eq (12)
                    xj = (Kj * Rcwj * (1 / depthp) * xp) + Kj * Tcwj; 
                    float depthj = depthp / (Rcwj[2] * xp + depthp * Tcwj[2]); 

                    // find the (float) coordinates of the new point in keyframe_j
                    cv::Mat xyzj = xj * cv::Mat(px, py, depthp);
                    float xj = xyzj[0];  
                    float yj = xyzj[1]; 
                    
                    int compatible_points_count = 0; // count of compatible neighbor pixels
                    std::vector<cv::Point> compatible_points;
                    // look in 4-neighborhood pixel p_j,n around xj for compatible inverse depth
                    for (int nj = floor(xj); nj <= nj + 1; nj++) {
                        for (int ny = floor(yj); ny < ny + 1; ny++) {
                            if (H[nx][ny] == NULL) continue;
                            float depthjn = H[nx][ny].depth; 
                            float sigmajn = H[nx][ny].sigma; 
                            // Eq (13)
                            float test = (depthp - depthjn) * (depthp - depthjn) / (sigmajn * sigmajn);
                            if (test < 3.84) {
                                compatible_points_count++;
                                compatible_points.push_back(cv::Point(nx, ny));
                            }
                        }
                    }
                    
                    // at least one compatible pixel p_j,n must be found in at least lambdaN neighbor keyframes
                    if (compatible_points_count) {
                        compatible_neighbor_keyframes_count++;
                        float depthp_star = 1000;
                        float sum_depth = 0;
                        for (int p; p < compatible_points.size(); p++) {
                            float depthjn = H[compatible_points[p].x][compatible_points[p].y].depth;
                            float sigmajn = H[compatible_points[p].x][compatible_points[p].y].sigma;
                            // equation (14)
                            sum_depth += pow((depthjn - depthp * Rcwj[2] * xp * Tcwj[2]), 2) / (pow(depthjn, 4) * pow(sigmajn, 2)); 
                        } 
                    } 
                }
                // don't retain the inverse depth distribution of this pixel if not enough support in neighbor keyframes
                if (compatible_neighbor_keyframes_count < lambdaN) {
                    H[px][py] = NULL;
                }
            }
        }
}
} 

/////////////////////
// utility methods //
/////////////////////

void getIntensityGradient_D(cv::Mat ImGrad, float* q) {
    float grad_d = (ImGrad.at<float>(uplusone,vplusone) - ImGrad.at<float>(uminone,vminone))/2;
    *q = grad_d;
} 

void getPixelDepth(cv::Mat& R, cv::Mat& T, ORB_SLAM::KeyFrame* kF cv::Mat& Im, int u, float *p) {
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

void getFusion(const vector<depthHo> best_compatible_ho, depthHo* hypothesis, float* min_sigma) {
    hypothesis->depth = 0;
    hypothesis->sigma = 0;
    float temp_min_sigma = 0;
    float pjsj =0; // numerator
    float rsj =0; // denominator
    for (int j = 0; j < best_compatible_ho.size(); j++) {
        pjsj += (best_compatible_ho[j].depth / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
        rsj += (1 / (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma));
        if (best_compatible_ho[j].sigma * best_compatible_ho[j].sigma < temp_min_sigma * temp_min_sigma) {
            temp_min_sigma = best_compatible_ho[j].sigma;
        }
    }
    hypothesis->depth = pjsj / rsj;
    hypothesis->sigma = sqrt(1 / rsj);
    if (min_sigma) {
        *min_sigma = temp_min_sigma;
    }
} 

// compute the chi_test values for neighbors of all pixels
void memoize_neighbor_support(const depthHo** H, const int& w, const int& h, std::vector<depthHo>** c) {
    c.clear();
    float chi = 0;
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            std::vector<float> temp;
            for (int ni = i - 1; ni <= i + 1; ni++) {
                for (int nj = j - 1; nj <= j + 1; nj++) {
                    if ((ni < 0) || (nj < 0)) continue;
                    if ((ni == i) && (nj == j)) continue;
                    if (chiTest(H[i][j], H[ni][nj], &chi))
                        temp.push_back(H[ni][nj]);
                }
            }
            c[i][j] = temp;
        }
    }
}
