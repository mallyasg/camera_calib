#include "cameraCalib.hpp"

void gsm::cameraCalib::findPattern(std::vector<cv::Mat>& images,
				   cv::Size boardSize,
				   std::vector<std::vector<cv::Point2f> >& imagePoints,
				   const int debug) {

  size_t index = 0;
  std::vector<cv::Mat>::iterator it;
  for (; index < images.size(); ) {
    cv::Mat image = images[index];
    std::vector<cv::Point2f> pointBuffer;
    bool patternFound;
    switch(mPattern) {
    case gsm::patternType::CHESSBOARD:
      patternFound = cv::findChessboardCorners(image,
					       boardSize,
					       pointBuffer,
					       cv::CALIB_CB_ADAPTIVE_THRESH |
					       cv::CALIB_CB_FAST_CHECK |
					       cv::CALIB_CB_NORMALIZE_IMAGE);
      break;
    case gsm::patternType::CIRCLES_GRID:
      patternFound = cv::findCirclesGrid(image,
					 boardSize,
					 pointBuffer);
      break;
    case gsm::patternType::ASYMMETRIC_CIRCLES_GRID:
      patternFound = cv::findCirclesGrid(image,
					 boardSize,
					 pointBuffer,
					 cv::CALIB_CB_ASYMMETRIC_GRID);
    default:
      std::cout << "Unknown Pattern\n";
      return;
    }
    
    if (mPattern == gsm::patternType::CHESSBOARD && patternFound) {
      cv::cornerSubPix(image,
		       pointBuffer,
		       cv::Size(11,11),
		       cv::Size(-1,-1),
		       cv::TermCriteria(cv::TermCriteria::EPS + \
					cv::TermCriteria::COUNT,
					30,
					0.1));
    }
    
    if (patternFound) {
      imagePoints.push_back(pointBuffer);
      if (debug) {
	cv::Mat displayImage;
	image.copyTo(displayImage);
	cv::drawChessboardCorners(displayImage,
	  boardSize,
	  cv::Mat(pointBuffer),
	  patternFound);

	cv::namedWindow("Chess Board Corners", CV_WINDOW_AUTOSIZE);
	cv::imshow("Chess Board Corners", displayImage);
	cv::waitKey(0);
	cv::imwrite("./results/image_" + std::to_string(index) + ".jpg", image);
      }
    index++;
    } else {
      std::swap(images[index], images[images.size() - 1]);
      images.pop_back();
    }
  }
}

void gsm::cameraCalib::calcObjectPoints(std::vector<std::vector<cv::Point3f> >& objectPoints,
					const int numImages,
					const cv::Size boardSize,
					const float squareSize) {
  
  std::vector<cv::Point3f> objectPointsTemp;
  switch(mPattern) {
  case gsm::patternType::CHESSBOARD:
  case gsm::patternType::CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; ++i) {
      for (int j = 0; j < boardSize.width; ++j) {
	objectPointsTemp.push_back(cv::Point3f(float(j * squareSize),
					   float(i * squareSize),
					   0.0));
      }
    }
    break;
  case gsm::patternType::ASYMMETRIC_CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; ++i) {
      for (int j = 0; j < boardSize.width; ++j) {
	objectPointsTemp.push_back(cv::Point3f(float((2 * j + i % 2) * squareSize),
					   float(i * squareSize),
					   0.0));
      }
    }
    break;
  default:
    CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
  }
  
  objectPoints.resize(numImages, objectPointsTemp); 
}

double gsm::cameraCalib::computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
						   const std::vector<std::vector<cv::Point2f> >& imagePoints,
						   const std::vector<cv::Mat>& rvecs,
						   const std::vector<cv::Mat>& tvecs,
						   std::vector<float>& perViewErrors) {
  std::vector<cv::Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0;
  double err = 0;
  perViewErrors.resize(objectPoints.size());
  
  for( i = 0; i < (int)objectPoints.size(); i++ )
    {
      // Project the 3D points i.e. the objectPoints using the camera
      // matrix obtained during camera calibration
      cv::projectPoints(cv::Mat(objectPoints[i]),
		    rvecs[i],
		    tvecs[i],
		    mCameraMatrix,
		    mDistCoeffs,
		    imagePoints2);

      // Compute the error between the projected image points and the points
      // obtained using findChessBoardCorners
      err = cv::norm(cv::Mat(imagePoints[i]),
		     cv::Mat(imagePoints2),
		     cv::NORM_L2);
      
      int n = (int)objectPoints[i].size();
      perViewErrors.push_back((float)std::sqrt(err*err/n));
      totalErr += err*err;
      totalPoints += n;
    }

  return std::sqrt(totalErr/totalPoints);
}

void gsm::cameraCalib::saveCalibrationResult(const std::string fileName,
					     const int flags,
					     const cv::Size imageSize,
					     const cv::Size boardSize,
					     const float squareSize,
					     const double totalAvgErr,
					     const std::vector<float> reprojErrs,
					     const std::vector<cv::Mat> rvecs,
					     const std::vector<cv::Mat> tvecs,
					     const std::vector<std::vector<cv::Point2f> > imagePoints) {
  
  cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
  
  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;
  fs << "board_width" << boardSize.width;
  fs << "board_height" << boardSize.height;
  fs << "square_size" << squareSize;

  fs << "flags" << flags;
  fs << "camera_matrix" << mCameraMatrix;
  fs << "distortion_coefficients" << mDistCoeffs;
  
  fs << "avg_reprojection_error" << totalAvgErr;
  if(!reprojErrs.empty()) {
    fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);
  }

  if(!rvecs.empty() && !tvecs.empty()) {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
    for(int i = 0; i < (int)rvecs.size(); i++)
      {
	cv::Mat r = bigmat(cv::Range(i, i+1), cv::Range(0,3));
	cv::Mat t = bigmat(cv::Range(i, i+1), cv::Range(3,6));
	
	CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
	CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
	//*.t() is MatExpr (not Mat) so we can use assignment operator
	r = rvecs[i].t();
	t = tvecs[i].t();
      }
    //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
    fs << "extrinsic_parameters" << bigmat;
  }

  if(!imagePoints.empty()){
    cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    for(int i = 0; i < (int)imagePoints.size(); i++)
      {
	cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
	cv::Mat imgpti(imagePoints[i]);
	imgpti.copyTo(r);
      }
    fs << "image_points" << imagePtMat;
  }
}

gsm::cameraCalib::cameraCalib() {
  mCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  mDistCoeffs = cv::Mat::zeros(8, 1, CV_64F);
  mPattern = gsm::patternType::CHESSBOARD;
}

gsm::cameraCalib::cameraCalib(cv::Mat cameraMatrix,
			      cv::Mat distCoeffs,
			      gsm::patternType pattern) {
  cameraMatrix.copyTo(mCameraMatrix);
  distCoeffs.copyTo(mDistCoeffs);
  mPattern = pattern;
}

void gsm::cameraCalib::runCalibration(std::vector<cv::Mat>& images,
				      const cv::Size boardSize,
				      const float squareSize,
				      const int debug) {
  
  std::vector<std::vector<cv::Point3f> > objectPoints;
  std::vector<std::vector<cv::Point2f> > imagePoints;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  std::vector<float> reprojErrs;
  const cv::Size imageSize = images[0].size();
  int flag = cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;
  
  // Detect the image points i.e. chess board corners in case of chess board
  findPattern(images, boardSize, imagePoints, debug);

  // Generate object Points
  calcObjectPoints(objectPoints,
		   (int)imagePoints.size(),
		   boardSize,
		   squareSize);

  // Calibrate cameras
  double rms = cv::calibrateCamera(objectPoints,
				   imagePoints,
				   imageSize,
				   mCameraMatrix,
				   mDistCoeffs,
				   rvecs,
				   tvecs,
				   flag,
				   cv::TermCriteria(cv::TermCriteria::COUNT + \
						    cv::TermCriteria::EPS,
						    30,
						    DBL_EPSILON
						    )   
				   );

  printf("RMS error reported by calibrateCamera: %g\n", rms);
  bool ok = cv::checkRange(mCameraMatrix) && cv::checkRange(mDistCoeffs);

  if (ok) {
    double totalAvgErr = computeReprojectionErrors(objectPoints,
						   imagePoints,
						   rvecs,
						   tvecs,
						   reprojErrs);
    
    std::cout << "Total Average Error : " << totalAvgErr << std::endl;
    std::string fileName = "./Calibration.yml";
    saveCalibrationResult(fileName,
			  flag,
			  imageSize,
			  boardSize,
			  squareSize,
			  totalAvgErr,
			  reprojErrs,
			  rvecs,
			  tvecs,
			  imagePoints);
    // Get optimal camera matrix
    mNewCameraMatrix = cv::getOptimalNewCameraMatrix(mCameraMatrix,
						     mDistCoeffs,
						     imageSize,
						     1.0,
						     imageSize,
						     &mROI);
  } else {
    std::cout << "Camera Matrix and Distortion co-efficients out of range\n";
  }
}

 void gsm::cameraCalib::undistortImage(std::vector<cv::Mat> images,
				       std::vector<cv::Mat>& undistortedImages,
				       int debug) {
   cv::Mat map1, map2;
   cv::initUndistortRectifyMap(mCameraMatrix,
			       mDistCoeffs,
			       cv::Mat(),
			       mNewCameraMatrix,
			       images[0].size(),
			       CV_16SC2,
			       map1,
			       map2);

   for (int i = 0; i < (int)images.size(); ++i) {
     cv::Mat tempImage, destImage;
     cv::remap(images[i], tempImage, map1, map2, cv::INTER_LINEAR);
     destImage = tempImage(mROI);
     if (debug) {
       cv::imwrite("./results/undistortedImage_" + std::to_string(i) + ".jpg",
		   destImage);
     }
     undistortedImages.push_back(destImage);
   }
 }

  void gsm::cameraCalib::undistortImage(cv::Mat image,
					cv::Mat& undistortedImage,
					int debug) {
   cv::Mat map1, map2;
   cv::initUndistortRectifyMap(mCameraMatrix,
			       mDistCoeffs,
			       cv::Mat(),
			       mNewCameraMatrix,
			       image.size(),
			       CV_16SC2,
			       map1,
			       map2);
   
   cv::Mat tempImage;
   cv::remap(image, tempImage, map1, map2, cv::INTER_LINEAR);
   undistortedImage = tempImage(mROI);
   if (debug) {
     cv::imwrite("./results/single_image/undistortedImage.jpg",
		 undistortedImage);
   }
 }
