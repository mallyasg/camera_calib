#include "cameraCalib.hpp"

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

void gsm::cameraCalib::runCalibration() {
  std::vector<std::vector<cv::Point3f> > objectPoints(1);
  
}

void gsm::cameraCalib::findPattern(std::vector<cv::Mat>& images,
				   cv::Size boardSize,
				   std::vector<std::vector<cv::Point2f> >& imagePoints) {

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
#ifdef DEBUG
      cv::Mat displayImage;
      image.copyTo(displayImage);
      cv::drawChessboardCorners(displayImage,
				boardSize,
				cv::Mat(pointBuffer),
				patternFound);

      cv::namedWindow("Chess Board Corners", CV_WINDOW_AUTOSIZE);
      cv::imshow("Chess Board Corners", displayImage);
      cv::waitKey(0);
      cv::imwrite("./results/left" + std::to_string(index) + ".jpg", image);
#endif
    index++;
    } else {
      std::swap(images[index], images[images.size() - 1]);
      images.pop_back();
    }
  }
}
