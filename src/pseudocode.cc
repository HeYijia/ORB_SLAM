
//parameters for semi-dense probability mapping
int N = 7;
int sigmaI = 20;
int lambdaG = 8;
int lambdaL = 80;
int lambdaTheta = 45;
int lambdaN = 3;
for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;
	
	
	//finding the Keyframes that best match pKF
	vector<ORB_SLAM::KeyFrame*> closestMatches = pKF->GetBestCovisibilityKeyFrames(20);
	set<MapPoint*> orbMP = pKF->GetMapPoints();
	vector<float> vDepths;
	vDepths.reserve(orbMP.size());
	cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
	Rcw2 = Rcw2.t();
	float zcw = Tcw_.at<float>(2,3);
	float maxDepth;
	float minDepth;

	for(size_t point=0; point<orbMP.size(); i++){
		if(orbMP[point]){
			MapPoint* pMP = orbMP[point];
			cv::Mat x3Dw = pMP->GetWorldPos();
			float z = Rcw2.dot(x3Dw)+zcw;
			vDepths.push_back(z);
		}
	}
	boost::variance::accumulator_set<double, stats<tag::variance> > acc;
	for_each(vDepths.begin(), vDepth.end(), bind<void>(ref(acc), _1));

	maxDepth = mean(acc) + 2*sqrt(variance(acc));
	minDepth = mean(acc) - 2*sqrt(variance(acc));

	cv::Mat pGradX, pGradY;
	cv::Mat original = pKF->GetImage();

	cv::Scharr(original, pGradX, CV_16S,1,0);
	cv::Scharr(original, pGradY, CV_16S,0,1);
	
	cv::Mat absPGradX, absPGradY;
	//cv::multiply(pGradX,pGradX,pGradX2);
	//cv::multiply(pGradY,pGradY,pGradY2);	
	//float pGrad = cv::sqrt(cv::multiply(pGradX,pGradX)+square(pGradY));
	cv::convertScaleAbs(pGardX,absPGradX);
	cv::convertScaleAbs(pGradY,absPGradY);
	cv::Mat pGrad;
       	cv::addWeighted(0.5,absPGradX,0.5,absPGradY,0,pGrad);
	

	for(size_t j=0; j<closestMatches.size(); i++){
		KeyFrame* pKF2 = closestMatches[j];
		cv::Mat image = pKF2->GetImage();
		cv::Mat sigmaI, mean;
		cv::meanStdDev(image,mean,sigmaI)
		for(int x = 0; x < image.rows; x++){
			for(int y = 0; y < image.cols; y++){
				if(pGrad.at<float>(x,y) < lambdaG){
					continue;
				}
				cv::Mat pj = image.at<Mat>(x,y);
				
				//calculate angle
				float valueX = absPGradX.at<float>(x,y);
				float valueY = absPGradY.at<float>(x,y);
				float thetap = cv::fastAtan2(valueX,valueY);
				
				cv::Mat F12 = LocalMapping::ComputeF12(pKF,pKF2);
				//could probably use the opencv built in function instead
				float a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
				float b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
				float c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
				//since a*x + b*y + c
				//x = uj
				//and we have y = (a/b)*x + (c/b)
				//actually let's try that
				float old_err = 1000.0;
				float best_ri = 0.0;
				float best_rg = 0.0;
				int best_pixel = 0;
				for(int uj = minDepth; uj < maxDepth; uj++){
					vj = (a/b)*uj + (c/b);
					cv::Mat pGrad2, pGradX2, pGradY2, absPGradX2, absPGradY2;

					cv::Scharr(image, pGradX2, CV_16S,1,0);
					cv::Scharr(image, pGradY2, CV_16S,0,1);
	
					cv::convertScaleAbs(pGardX2,absPGradX2);
					cv::convertScaleAbs(pGradY2,absPGradY2);
       					cv::addWeighted(0.5,absPGradX2,0.5,absPGradY2,0,pGrad2);
					if(pGrad2.at<float>(uj,vj) < lambdaG){
						continue;
					}
					float valueX2 = absPGradX2.at<float>(i,j);
					float valueY2 = absPGradY2.at<float>(i,j);
					float thetaUj = cv::fastAtan2(valueX,valueY);

					float thetaL = cv::fastAtan2(uj,vj);
						
					if(std::abs(thetaUj - thetaL + M_PI) < lambdaTheta)
						continue;
					if(std::abs(thetaUj - thetaL - M_PI) < lambdaTheta)
						continue;
					//if(std::abs(thetaUj - (thetap + deltaTheta)) < lambdaTheta)
					//	continue;		
					float ri = original.at<float>(x,y) - pj;
					float rg = pGrad - pGrad2;
			     		float err = (ri*ri + (rg*rg)/0.23)/(sigmaI*sigmaI);
					if(err < old_err){
						best_pixel = uj;
						old_err = err;
						best_ri = ri;
						best_rg = rg;
					}
				}

				int uplusone = best_pixel + 1; 
				int vplusone = (a/b)*uplusone + (c/b);
				int uminone = best_pixel - 1; 
				int vminone = (a/b)*uminone + (c/b);
				
				float g = (image.at<float>(uplusone,vplusone) -image.at<float>(uminone,vminone))/2;  

				float q = (pGrad2.at<float>(uplusone,vplusone) - pGrad2.at<float>(uminone,vminone))/2;
				
				float ustar = best_pixel + (g*best_ri + (1/0.23)*q*best_rg)/(g*g + (1/0.23)*q*q);

				float sigma_ustar_sqr = (2*sigmaI*sigmaI)/(g*g + q*q);

				cv::Mat Rcw2 = pKF2->GetRotation();
				cv::Mat Rwc2 = Rcw2.t();
			        cv::Mat tcw2 = pKF2->GetTranslation();
				cv::Mat Tcw2(3,4,CV_32F);
				Rcw2.copyTo(Tcw2.colRange(0,3));
				tcw2.copyTo(Tcw2.col(3));


				const float fx2 = pKF2->fx;
			        const float cx2 = pKF2->cx;

				cv::Mat K2 = pKF2 -> GetCalibrationMatrix;
				
				cv::Mat xp = K2*image;
				int ujcx = best_pixel - cx2;
				int vjcx = (a/b)*ujcx + (c/b);
				float depthp = (Rcw2[2]*xp.at<float>(ujcx,vjcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ujcx][vjcx]+fx2*Tcw2);
			
				int ustarcx = ustar - cx2;
				int vstarcx = (a/b)*ustarcx + (c/b);
				
				float depthj = (Rcw2[2]*xp.at<float>(ustarcx,vstarcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ustarcx][vstarcx]+fx2*Tcw2);

				int ustarcx = ustar - cx2;
				int vstarcx = (a/b)*ustarcx + (c/b);
				
				float depthj = (Rcw2[2]*xp.at<float>(ustarcx,vstarcx)-fx2*Rcw2[0]*xp)/(-Tcw2[2][ustarcx][vstarcx]+fx2*Tcw2);
			


			}
		}
	}


