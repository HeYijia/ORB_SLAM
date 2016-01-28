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


void firstLoop(KeyFrame kf, depthHo**, std::vector<depthHo>&){;}

void stereoSearch_constraints(){;}
	        
void epipolarSearch(){;}

////////////////////////
// Utility functions
////////////////////////

void getImageGradient(cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* grad){
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

void getGradientOrientation(int x, int y, cv::Mat& gradx, cv::Mat& grady, float th){
  float valuex = gradx.at<float>(x,y);
  float valuey = grady.at<float>(x,y);
  float th =  cv::fastAtan2(gradx,grady);
}

//might be a good idea to store these when they get calculated during ORB-SLAM.
void getInPlaneRotation(KeyFrame& k1, KeyFrame& k2, float th){
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
