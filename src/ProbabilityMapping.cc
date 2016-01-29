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
 *         Author:  Josh Tang, Rebecca Frederic 
 *   Organization:  
 *
 * =====================================================================================
 */
#include "ProbabilityMapping.h"
#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include <vector>


void firstLoop(KeyFrame kf, depthHo** ho, std::vector<depthHo>* depth_ho){
  vector<KeyFrame*> closestMatches = kf -> GetBestCovisibilityFrames();
  float max_depth;
  float min_depth;
  StereoSearchConstraints(kf, min_depth, max_depth);
  
  cv::Mat gradx, grady, grad;
  cv::Mat image = kf->GetImage();
  GetImageGradient(image,gradx,grady,grad);
  
  vector<depthHo> depth_ho;
  depthHo ho[image.rows][image.cols];

  for(int x = 0; x < image.rows; x++){
    for(int y = 0; y < image.cols; y++){
      ho[x][y] = NULL;
      if(grad.at<float>(x,y) < lambdaG)
        continue;
  
      for(size_t i=0; i<closestMatches.size(); i++){
        KeyFrame* kf2 = closestMatches[j];
        
        struct depthHo dh;
        EpipolarSearch(kf,kf2,x,y,gradx,grady,grad,min_depth,max_depth,dh);
        depth_ho.push_back(dh);
        ho[x][y] = dh;
      }
    }
  }
}

void StereoSearchConstraints(KeyFrame kf, float* min_depth, float* max_depth){
  
  vector<float> orb_depths = kf->GetAllPointDepths();
  
  boost::variance::accumulator_set<double, stats<tag::variance> > acc;
  for_each(orb_depths.begin(), orb_depths.end(), bind<void>(ref(acc), _1));

  float max_depth = mean(acc) + 2*sqrt(variance(acc));
  float min_depth = mean(acc) - 2*sqrt(variance(acc));
}
	        
void EpipolarSearch(KeyFrame kf1, Keyframe kf2, int x, int y, cv::Mat gradx, cv::Mat grady, cv::Mat grad, float min_depth, float max_depth, depthHo* dh){
  cv::Mat original = kf1->GetImage();
  cv::Mat pixel = original.at<cv::Mat>(x,y);

  cv::Mat image = kf2 -> GetImage();
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

////////////////////////
// Utility functions
////////////////////////

void ComputeInvDepthHypothesis(KeyFrame kf, int pixel, float ustar, float ustar_var, float a, float b, float c, depthHo* dh){
  cv::Mat image = kf -> GetImage();

  cv::Mat frame_rot = kf->GetRotation();
  cv::Mat inv_frame_rot =  frame_rot.t();
  cv::Mat frame_translation = kf -> GetTranslation();
  
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

  float sigma_depth = max(abs(inv_depth_max),abs(inv_depth_min));

  dh.depth = inv_pixel_depth;
  dh.sigma = sigma_depth;
}

void GetImageGradient(cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* grad){
  cv::Mat gradx, grady;
	
  cv::Scharr(image, gradx, CV_16S,1,0);
  cv::Scharr(image, grady, CV_16S,0,1);
	
  cv::Mat absgradx, absgrady;

  cv::convertScaleAbs(gradx, absgradx);
  cv::convertScaleAbs(grady, absgrady);
  gradx = absgradx;
  grady = absgrady;
  cv::addWeighted(absgradx,0.5,absgrady,0.5,grad,0);
}

void GetGradientOrientation(int x, int y, cv::Mat& gradx, cv::Mat& grady, float th){
  float valuex = gradx.at<float>(x,y);
  float valuey = grady.at<float>(x,y);
  float th =  cv::fastAtan2(gradx,grady);
}

//might be a good idea to store these when they get calculated during ORB-SLAM.
void GetInPlaneRotation(KeyFrame& k1, KeyFrame& k2, float th){
  vector<cv::KeyPoint> vKPU1 = k1->GetKeyPointsUn();
  DBoW2::FeatureVector vFeatVec1 = k1->GetFeatureVector();
  vector<MapPoint*> vMapPoints1 = k1->GetMapPointMatches();
  cv::Mat Descriptors1 = k1->GetDescriptors();

  vector<cv::KeyPoint> vKPU2 = k2->GetKeyPointsUn();
  DBoW2::FeatureVector vFeatVec2 = k2->GetFeatureVector();
  vector<MapPoint*> vMapPoints2 = k2 ->GetMapPointMatches();
  cv::Mat Descriptors2 = k2->GetDescriptors();

  vector<int> rotHist[histo_length];
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
    th = (rotHist[size/2 - 1] + rotHist[size/2])/2;
  }
  else{
    th = rotHist[size/2];
  }
  return th;
}
