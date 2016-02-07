/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.cc
 *
 *    Description:  
 *
 *        Version:  0.1
 *        Created:  01/21/2016 10:39:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Josh Tang, Rebecca Frederick
 *   Organization:  Unkei 
 *
 * =====================================================================================
 */

#include <cmath>
#include <opencv2/opencv.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include "ProbabilityMapping.h"
#include "KeyFrame.h"
#include "LocalMapping.h"

ProbabilityMapping::ProbabilityMapping() {}
void ProbabilityMapping::ComputeInvDepthHypothesis(ORB_SLAM::KeyFrame* kf, int pixel, float ustar, float ustar_var, float a, float b, float c, depthHo& dh) {}
//void ProbabilityMapping::GetImageGradient(const cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* grad) {}
//void ProbabilityMapping::GetGradientOrientation(int x, int y, const cv::Mat& gradx, const cv::Mat& grady, float* th) {}
void ProbabilityMapping::GetInPlaneRotation(ORB_SLAM::KeyFrame* k1, ORB_SLAM::KeyFrame* k2, float* th) {}
void ProbabilityMapping::GetIntensityGradient(cv::Mat im, float* g) {}
void ProbabilityMapping::PixelNeighborSupport(depthHo*** H, int x, int y, std::vector<depthHo*>* support) {}
void ProbabilityMapping::PixelNeighborNeighborSupport(depthHo*** H, int px, int py, std::vector<std::vector<depthHo*> >* support) {}
void ProbabilityMapping::GetIntensityGradient_D(const cv::Mat& im, float* q) {}
//void ProbabilityMapping::GetPixelDepth(const cv::Mat& Im, const cv::Mat& R, const cv::Mat& T, ORB_SLAM::KeyFrame* kF, int u, float *p) {}
//bool ProbabilityMapping::ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val) { return true; }
//void ProbabilityMapping::GetFusion(const std::vector<depthHo*>& best_compatible_ho, depthHo* hypothesis, float* min_sigma) {}

//void ProbabilityMapping::FirstLoop(ORB_SLAM::KeyFrame *kf, depthHo*** ho, std::vector<depthHo*>* depth_ho);
//void ProbabilityMapping::StereoSearchConstraints(ORB_SLAM::KeyFrame *kf, float* min_depth, float* max_depth) {}
void ProbabilityMapping::EpipolarSearch(ORB_SLAM::KeyFrame *kf1, ORB_SLAM::KeyFrame *kf2, int x, int y, cv::Mat gradx, cv::Mat grady, cv::Mat grad, float min_depth, float max_depth, depthHo* dh) {}
//void ProbabilityMapping::InverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo* dist) {}
//void ProbabilityMapping::IntraKeyFrameDepthChecking(depthHo** h, int imrows, int imcols) {}
//void ProbabilityMapping::InterKeyFrameDepthChecking(ORB_SLAM::KeyFrame* currentKF, depthHo** h, int imrows, int imcols) {}





//  depthHo ho[image.rows][image.cols];
void ProbabilityMapping::FirstLoop(ORB_SLAM::KeyFrame *kf, depthHo*** ho, std::vector<depthHo*>* depth_ho){
  
  depth_ho->clear();
  
  std::vector<ORB_SLAM::KeyFrame*> closestMatches = kf->GetBestCovisibilityKeyFrames(covisN);
  
  float max_depth;
  float min_depth;
  StereoSearchConstraints(kf, &min_depth, &max_depth);
  
  cv::Mat gradx, grady, grad;
  cv::Mat image = kf->GetImage();
  GetImageGradient(image, &gradx, &grady, &grad);
  
  for(int x = 0; x < image.rows; x++){
    for(int y = 0; y < image.cols; y++){
      ho[x][y] = NULL;
      if(grad.at<float>(x,y) < lambdaG)
        continue;
  
      for(size_t i=0; i<closestMatches.size(); i++){
        ORB_SLAM::KeyFrame* kf2 = closestMatches[i];
        
        struct depthHo* dh;
        EpipolarSearch(kf, kf2, x, y, gradx, grady, grad, min_depth, max_depth, dh);
        depth_ho->push_back(dh);
        ho[x][y] = dh;
      }
    }
  }
}

void StereoSearchConstraints(ORB_SLAM::KeyFrame* kf, float* min_depth, float* max_depth){
  
  std::vector<float> orb_depths = kf->GetAllPointDepths();
  
  accumulator_set<double, stats<tag::variance> > acc;
  for_each(orb_depths.begin(), orb_depths.end(), bind<void>(ref(acc), _1));

  *max_depth = mean(acc) + 2*sqrt(variance(acc));
  *min_depth = mean(acc) - 2*sqrt(variance(acc));
}

/*  
void EpipolarSearch(ORB_SLAM::KeyFrame *kf1, const KeyFrame *kf2, int x, int y, cv::Mat gradx, cv::Mat grady, cv::Mat grad, float min_depth, float max_depth, depthHo* dh){
  cv::Mat original = kf1->GetImage();
  cv::Mat pixel = original.at<cv::Mat>(x,y);

  cv::Mat image = kf2->GetImage();
  cv::Mat image_stddev, image_mean;
  cv::meanStdDev(image,mean,image_stddev);
  
  cv::Mat F12 = LocalMapping::ComputeF12(kf1,kf2); 
  float a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
  float b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
  float c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
  
  float old_err = 1000.0;
  float best_photometric_err = 0.0;
  float best_gradient_modulo_err = 0.0;
  int best_pixel;

  for(int uj = minDepth; uj < maxDepth; uj++){
    vj = (a/b)*uj+(c/b);
    
    float th_grad, th_epipolar_line, th_pi, th_rot;
    cv::Mat gradx2, grady2, grad2;
    GetImageGradient(image, gradx2, grady2, grad2);
    GetGradientOrientation(uj,vj,gradx2,grady2,th_grad);
    th_epipolar_line = fastAtan2(uj,vj); 
    GetInPlaneRotation(kf1, kf2, th_rot);
    GetImageGradient(x,y,gradx,grady,th_pi);

    if(grad2.at<float>(uj,vj) < lambdaG)
      continue;
    if(abs(th_grad - th_epipolar_line + M_PI) < lambdaG)
      continue;
    if(abs(th_grad - th_epipolar_line - M_PI) < lambdaG)
      continue;
    if(abs(th_grad - ( th_pi + th_rot )) < lambdaTheta)
      continue;
    
    float photometric_err = pixel - image.at<cv::Mat>(uj,vj);
    float gradient_modulo_err = grad - grad2;
    float err = (photometric_err*photometric_err + (gradient_modulo_err*gradient_modulo_err)/0.23)/(image_stddev);

    if(err < old_err){
      best_pixel = uj;
      old_err = err;
      best_photometric_err = photometric_err;
      best_gradient_modulo_err = gradient_modulo_err;
    }
  }

  int uj_plus = best_pixel + 1;
  int vj_plus = (a/b)*uj_plus + (c/b);
  int uj_minus = best_pixel - 1;
  int vj_minus = (a/b)*uj_minus + (c/b);

  float g = (image.at<float>(uj_plus, vj_plus) - image.at<float>(uj_minus, vj_minus))/2.0;

  float q = (grad2.at<float>(uj_plus, uv_plus) - grad2.at<float>(uj_minus, vj_plus))/2;

  float ustar = best_pixel + (g*best_photometric_err + (1/0.23)*q*best_gradinet_modulo_err)/(g*g + (1/0.23)*q*q);
  float ustar_var = (2*image_stddev*image_stddev)/(g*g

  ComputeInvDepthHypothesis(kf, best_pixel, ustar, ustar_var, a, b, c, dh);
  
  }
}
*/
void ProbabilityMapping::IntraKeyFrameDepthChecking(depthHo*** ho, int imrows, int imcols) {

    depthHo* ho_new[imrows][imcols];
    for (int px = 1; px < (imrows - 1); px++) {
        for (int py = 1; py < (imcols - 1); py++) {
            ho_new[px][py] = NULL;
            if (ho[px][py] == NULL) {
                // check if this pixel is surrounded by at least two pixels that are compatible to each other.
                std::vector<std::vector<depthHo*> > compatible_neighbor_neighbor_ho;
                
                PixelNeighborNeighborSupport(ho, px, py, &compatible_neighbor_neighbor_ho);
                
                unsigned int max_support = 0;
                unsigned int max_support_index = 0;
                for (size_t c = 0; c < compatible_neighbor_neighbor_ho.size(); c++) {
                    if (compatible_neighbor_neighbor_ho[c].size() > max_support) {
                        max_support = compatible_neighbor_neighbor_ho[c].size();
                        max_support_index = c;
                    }
                }
                
                // potentially grow the reconstruction density
                if (max_support >= 2) {
                    // assign this previous NULL depthHo the average depth and min sigma of its compatible neighbors
                    depthHo* fusion;
                    float min_sigma;

                    GetFusion(compatible_neighbor_neighbor_ho[max_support_index], fusion, &min_sigma);

                    ho_new[px][py]->depth = fusion->depth;
                    ho_new[px][py]->sigma = min_sigma;
                }

            } else {
                // calculate the support of the pixel's  8 neighbors
                std::vector<depthHo*> compatible_neighbor_ho;
                
                PixelNeighborSupport(ho, px, py, &compatible_neighbor_ho);
                
                if (compatible_neighbor_ho.size() < 2) {
                    // average depth of the retained pixels
                    // set sigma to minimum of neighbor pixels
                    depthHo fusion;
                    float min_sigma = 0;

                    GetFusion(compatible_neighbor_ho, &fusion, &min_sigma);

                    ho_new[px][py]->depth = fusion.depth;
                    ho_new[px][py]->sigma = min_sigma;

                } else {
                    ho_new[px][py] = ho[px][py];
                }
            }
        }
    }
    
    for (int x = 0; x < imrows; x++) {
        for (int y = 0; y < imcols; y++) {
            ho[x][y] = ho_new[x][y];
        }
    }
} 

void ProbabilityMapping::InverseDepthHypothesisFusion(const std::vector<depthHo*>& h, depthHo* dist) {
    dist->depth = 0;
    dist->sigma = 0;

    std::vector<depthHo*> compatible_ho;
    std::vector<depthHo*> compatible_ho_temp;
    float chi = 0;
    
    for (size_t a=0; a < h.size(); a++) {
        compatible_ho_temp.clear();
        
        for (size_t b=0; b < h.size(); b++) {
            // test if the hypotheses a and b are compatible
            if (ChiTest(*(h[a]), *(h[b]), &chi)) {
                compatible_ho_temp.push_back(h[b]); 
            }
        }
        // test if hypothesis 'a' has the required support
        if (compatible_ho_temp.size()-1 >= lambdaN && compatible_ho_temp.size() > compatible_ho.size()) {
            compatible_ho_temp.push_back(h[a]); 
            compatible_ho = compatible_ho_temp;
        }
    }

    // calculate the parameters of the inverse depth distribution by fusing hypotheses
    if (compatible_ho.size() >= lambdaN) {
        GetFusion(compatible_ho, dist, &chi);
    }
} 
/*
void ProbabilityMapping::InterKeyFrameDepthChecking(const cv::Mat& im, ORB_SLAM::KeyFrame* currentKf, depthHo*** h) {//int imrows, int imcols) {
        std::vector<ORB_SLAM::KeyFrame*> neighbors;
        
        // option1: could just be the best covisibility keyframes
        neighbors = currentKf->GetBestCovisibilityKeyFrames(covisN);
        
        // option2: could be found in one of the LocalMapping SearchByXXX() methods
        //ORB_SLAM::LocalMapping::SearchInNeighbors(); //mpCurrentKeyFrame->updateConnections()...AddConnection()...UpdateBestCovisibles()...
        //ORB_SLAM::LocalMapping::mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(covisN); //mvpOrderedConnectedKeyFrames()
        
        // for each pixel of keyframe_i, project it onto each neighbor keyframe keyframe_j
        // and propagate inverse depth
        for (int px = 0; px < im.rows; px++) {
            for (int py = 0; py < im.cols; py++) {
                if (h[px][py] == NULL) continue; 
                
                float depthp = h[px][py]->depth;
                int compatible_neighbor_keyframes_count = 0; // count of neighboring keyframes in which there is at least one compatible pixel
	        
                for(size_t j=0; j<neighbors.size(); j++){ 
		    ORB_SLAM::KeyFrame* pKFj = neighbors[j];
                    
                    // calibration matrix
	            cv::Mat Kj = currentKf->GetCalibrationMatrix();
		    cv::Mat Xp = Kj*im;
                    
                    // Tcw_ matrix FIXME: check if this is correct
                    cv::Mat Tcw_ = currentKf->GetTranslation(); 
                    // rotation matrix
                    cv::Mat Rcwj = Tcw_.row(2).colRange(0,3);
                    Rcwj = Rcwj.t();
                    
                    // translation matrix
                    cv::Mat tcwj = pKFj->GetTranslation();
                    cv::Mat Tcwj(3,4,CV_32F);
                    Rcwj.copyTo(Tcwj.colRange(0,3));
                    tcwj.copyTo(Tcwj.col(3));
                    
                    // compute the projection matrix to map 3D point from original image to 2D point in neighbor keyframe
                    // Eq (12)
                    cv::Mat Xj = (Kj * Rcwj * (1 / depthp) * Xp) + Kj * Tcwj; 
                    float depthj = depthp / (Rcwj.row(2).colRange(0,3) * Xp + depthp * Tcwj.row(2).colRange(0,3)); 

                    // find the (float) coordinates of the new point in keyframe_j
                    cv::Mat pointKfj = Xj * cv::Mat(px, py, depthp);
                    float xj = pointKfj.at<float>(0,0); 
                    float yj = pointKfj.at<float>(1,0);
                    
                    int compatible_points_count = 0; // count of compatible neighbor pixels
                    std::vector<cv::Point> compatible_points;
                    // look in 4-neighborhood pixel p_j,n around xj for compatible inverse depth
                    int xj_floor = floor(xj);
                    int yj_floor = floor(yj);
                    for (int nx = xj_floor; nx <= xj_floor + 1; nx++) {
                        for (int ny = yj_floor; ny < yj_floor + 1; ny++) {
                            if (h[nx][ny] == NULL) continue;
                            float depthjn = h[nx][ny]->depth; 
                            float sigmajn = h[nx][ny]->sigma; 
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
                        for (size_t p; p < compatible_points.size(); p++) {
                            float depthjn = h[compatible_points[p].x][compatible_points[p].y]->depth;
                            float sigmajn = h[compatible_points[p].x][compatible_points[p].y]->sigma;
                            // equation (14)
                            sum_depth += pow((depthjn - depthp * Rcwj.row(3).colRange(0,3) * Xp * Tcwj.row(2).colRange(0,3)), 2) / (pow(depthjn, 4) * pow(sigmajn, 2)); 
                        } 
                    } 
                }
                // don't retain the inverse depth distribution of this pixel if not enough support in neighbor keyframes
                if (compatible_neighbor_keyframes_count < lambdaN) {
                    h[px][py] = NULL;
                }
            }
        }
    }
} 
*/



////////////////////////
// Utility functions
////////////////////////
/*
void ComputeInvDepthHypothesis(ORB_SLAM::KeyFrame* kf, int pixel, float ustar, float ustar_var, float a, float b, float c, depthHo& dh) {
  cv::Mat image = kf->GetImage();

  cv::Mat frame_rot = kf->GetRotation();
  cv::Mat inv_frame_rot =  frame_rot.t();
  cv::Mat frame_translation = kf->GetTranslation();
  
  cv::Mat transform_data(3,4,CV_32F);
  frame_rot.copytTo(transform_data.colRange(0,3));
  frame_translation.copyTo(transform_data.col(3));

  const float fx = kf -> fx;
  const float cx = kf -> cx;
  cv::Mat calibration_matrix = kf -> GetCalibrationMatrix();
  
  cv::Mat corrected_image = calibrated_matrix * image;
  int ujcx = pixel - cx;
  int vjcx = (a/b) * ujcx + (c/b);

  float inv_pixel_depth = (inv_frame_rot[2]*corrected_image.at<float>(ujcx,vjcx)-fx*inv_frame_rot[0]*corrected_image)/(transform_data[2][ujcx][vjcx]+fx*transform_data[0]);
  
  int ustarcx_min = ustar - cx - sqrt(ustar_var);
  int vstarcx_min = (a/b)*ustarcx_min + (c/b);

  float inv_depth_min = (inv_frame_rot[2]*corrected_image.at<float>(ustarcx_min ,vstarcx_min)-fx*inv_frame_rot[0]*corrected_image)/(-transform_data[2][ustarcx_min][vstarcx_min]+fx*transform_data[0]); 
  
  int ustarcx_max = ustar - cx + sqrt(ustar_var);
  int vstarcx_max = (a/b)*ustarcx_max + (c/b);
  
  float inv_depth_max = (inv_frame_rot[2]*corrected_image.at<float>(ustarcx_max ,vstarcx_max)-fx*inv_frame_rot[0]*corrected_image)/(-transform_data[2][ustarcx_max][vstarcx_max]+fx*transform_data[0]);

  float sigma_depth = max(abs(inv_depth_max), abs(inv_depth_min));

  dh.depth = inv_pixel_depth;
  dh.sigma = sigma_depth;
}
*/
void ProbabilityMapping::GetImageGradient(const cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* grad) {
  cv::Scharr(image, *gradx, CV_16S, 1, 0);
  cv::Scharr(image, *grady, CV_16S, 0, 1);
	
  cv::Mat absgradx, absgrady;

  cv::convertScaleAbs(*gradx, absgradx);
  cv::convertScaleAbs(*grady, absgrady);
  
  *gradx = absgradx;
  *grady = absgrady;

  cv::addWeighted(absgradx, 0.5, absgrady, 0.5, 0, *grad);
}

void ProbabilityMapping::GetGradientOrientation(int x, int y, const cv::Mat& gradx, const cv::Mat& grady, float* th){
  float valuex = gradx.at<float>(x,y);
  float valuey = grady.at<float>(x,y);
  
  *th =  cv::fastAtan2(valuex, valuey);
}

/*
//might be a good idea to store these when they get calculated during ORB-SLAM.
void GetInPlaneRotation(ORB_SLAM::KeyFrame* k1, ORB_SLAM::KeyFrame* k2, float* th) {
  std::vector<cv::KeyPoint> vKPU1 = k1->GetKeyPointsUn();
  DBoW2::FeatureVector vFeatVec1 = k1->GetFeatureVector();
  std::vector<MapPoint*> vMapPoints1 = k1->GetMapPointMatches();
  cv::Mat Descriptors1 = k1->GetDescriptors();

  std::vector<cv::KeyPoint> vKPU2 = k2->GetKeyPointsUn();
  DBoW2::FeatureVector vFeatVec2 = k2->GetFeatureVector();
  std::vector<MapPoint*> vMapPoints2 = k2 ->GetMapPointMatches();
  cv::Mat Descriptors2 = k2->GetDescriptors();

  std::vector<int> rotHist[histo_length];
  for(int i=0;i<histo_length;i++)
    rotHist[i].reserve(500);DescriptorDistance
  
  const float factor = 1.0f/histo_length;

  DBoW2::FeatureVector::iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::iterator f2end = vFeatVec2.end();

  while(f1it != f1end && fit != f2end) {
    if(f1it->first == f2it->first){
      for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++){
        size_t index1 = f1it->second[i1];

        MapPoint* pMP1 = vpMapPoints1[index1];
        if(!pMP1)
          continue;
        if(pMP1->isBad())
          continue;

        cv::Mat d1 = Descriptors1.row(index1);

        int bestDist1 = INT_MAX;
        int bestIndex2 = -1;
        int bestDist2 = INT_MAX;

        for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++){
          size_t index2 = f2it->second[i2];

          MapPoint* pMP2 = vpMapPoints2[index2];
          if(!pMP2)
            continue;
          if(pMP2->isBad())
            continue;

          cv::Mat d2 = Descriptors.row(index2);

          int dist = ORBmatcher::DescriptorDistance(d1,d2);

          if(dist<bestDist1){
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIndex2 = index2;
          }
          else if(dist<bestDist2){
            bestDist2 = dist;
          }
        }
        if(bestDist1<th_low){
          if(static_cast<float>(bestDist1)<nnratio*static_cast<float>(bestDist2)){
            float rot = vKPU1[index1].angle - vKPU2[index2].angle;
            if(rot<0.0)
              rot+=360.0f;
            int bin = round(rot*factor);
            if(bin==histo_length)
              bin=0;
            rotHist[bin].push_back(index1);
          }
        }
      }
    }
  }
  //calculate the median angle
  size_t size = rotHist.size();
  std::sort(rotHist.begin(),rotHist.end());

  if(size % 2 == 0){
    *th = (rotHist[size/2 - 1] + rotHist[size/2])/2;
  }
  else{
    *th = rotHist[size/2];
  }
}
*/
/*
void ProbabilityMapping::PixelNeighborSupport(const depthHo*** H, int px, int py, std::vector<depthHo>* support) {
    support->clear();
    for (int x = px - 1; x <= px + 1; x++) {
        for (int y = py - 1; y <= py + 1; y++) {
            if (x == px && y == py) continue; 
            if (chi_test(H[x][y], H[px][py], NULL)) {
                support->push_back(H[px][py]);
            }
        }
    }
}
*/
/*
void ProbabilityMapping::PixelNeighborNeighborSupport(const depthHo** H, int px, int py, std::vector<std::vector<depthHo>* support) {
    support->clear();
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
            support->push_back(tempSupport);
        }
    }
}
*/
/*
void ProbabilityMapping::GetIntensityGradient_D(const cv::Mat& ImGrad, float* q) {
    float grad_d = (ImGrad.at<float>(uplusone,vplusone) - ImGrad.at<float>(uminone,vminone))/2;
    *q = grad_d;
} 
*/

void ProbabilityMapping::GetTR(ORB_SLAM::KeyFrame* kf, cv::Mat* t, cv::Mat* r) {
    
    cv::Mat Rcw2 = kf->GetRotation();
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat tcw2 = kf->GetTranslation();
    cv::Mat Tcw2(3,4,CV_32F);
    Rcw2.copyTo(Tcw2.colRange(0,3));
    tcw2.copyTo(Tcw2.col(3));
    
    *t = Tcw2;
    *r = Rcw2;
}

void ProbabilityMapping::GetParameterization(const cv::Mat& F12, const int x, const int y, float* a, float* b, float* c) {
    // parameterization of the fundamental matrix (function of horizontal coordinate)
    // could probably use the opencv built in function instead
    *a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
    *b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
    *c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
}

/*  
// Equation (8)
oid ProbabilityMapping::GetPixelDepth(const float a, const float b, const float c, const int px, const cv::Mat& im, ORB_SLAM::KeyFrame* kf, float* p) {
        
    cv::Mat rcw(3,4,CV_32F);
    cv::Mat tcw(3,4,CV_32F);
    GetTR(kf, &rcw, &tcw);
    
    cv::Mat k = kf->GetCalibrationMatrix();
    cv::Mat xp = k*im;

    float fx = kf->fx;
    float cx = kf->cx;

    int ucx = px - cx;
    //int vcx = (a/b)*ucx + (c/b);
    
    *p = (((rcw.row(2) * xp) * ucx) - (fx * (rcw.row(0) * xp))) / ((-tcw.at<float>(2,0) * ucx) + (fx * tcw.at<float>(0,0)));
    // *p = (rcw[2] * xp.at<float>(ucx,vcx) - fx * rcw[0] * xp) / (-tcw[2][ucx][vcx] + fx * tcw[0]);
} 
*/
bool ProbabilityMapping::ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val) {
    float chi_test = (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma) + (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma);
    if (chi_val)
        *chi_val = chi_test;
    return (chi_test < 5.99);
} 

void ProbabilityMapping::GetFusion(const std::vector<depthHo*>& compatible_ho, depthHo* hypothesis, float* min_sigma) {
    hypothesis->depth = 0;
    hypothesis->sigma = 0;
    
    float temp_min_sigma = 0;
    float pjsj =0; // numerator
    float rsj =0; // denominator
    
    for (size_t j = 0; j < compatible_ho.size(); j++) {
        pjsj += (compatible_ho[j]->depth / (compatible_ho[j]->sigma * compatible_ho[j]->sigma));
        rsj += (1 / (compatible_ho[j]->sigma * compatible_ho[j]->sigma));
        if (compatible_ho[j]->sigma * compatible_ho[j]->sigma < temp_min_sigma * temp_min_sigma) {
            temp_min_sigma = compatible_ho[j]->sigma;
        }
    }
    
    hypothesis->depth = pjsj / rsj;
    hypothesis->sigma = sqrt(1 / rsj);
    if (min_sigma) {
        *min_sigma = temp_min_sigma;
    }
} 

