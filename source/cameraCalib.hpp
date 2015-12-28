#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>

#define DEBUG

namespace gsm {
  enum class patternType {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};
  class cameraCalib {
  private:
    cv::Mat          mCameraMatrix;
    cv::Mat          mDistCoeffs;
    patternType      mPattern;
  public:
    cameraCalib();
    cameraCalib(cv::Mat cameraMatrix, cv::Mat distCoeffs, patternType pattern);
    void findPattern(std::vector<cv::Mat>& images,
		     cv::Size boardSize,
		     std::vector<std::vector<cv::Point2f> >& imagePoints);
    void runCalibration();
    
  };
}
