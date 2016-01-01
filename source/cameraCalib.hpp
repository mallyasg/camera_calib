#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>

namespace gsm {
  enum class patternType {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};
  class cameraCalib {
  private:
    cv::Mat          mCameraMatrix;
    cv::Mat          mDistCoeffs;
    patternType      mPattern;
    cv::Rect         mROI;
    cv::Mat          mNewCameraMatrix;
  protected:
    void findPattern(std::vector<cv::Mat>& images,
		     cv::Size boardSize,
		     std::vector<std::vector<cv::Point2f> >& imagePoints,
		     const int debug);
    void calcObjectPoints(std::vector<std::vector<cv::Point3f> >& objectPoints,
			  const int numImages,
			  const cv::Size boardSize,
			  const float squareSize);
    
    double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
				     const std::vector<std::vector<cv::Point2f> >& imagePoints,
				     const std::vector<cv::Mat>& rvecs,
				     const std::vector<cv::Mat>& tvecs,
				     std::vector<float>& perViewErrors);
    
    void saveCalibrationResult(const std::string fileName,
					     const int flags,
					     const cv::Size imageSize,
					     const cv::Size boardSize,
					     const float squareSize,
					     const double totalAvgErr,
					     const std::vector<float> reprojErrs,
					     const std::vector<cv::Mat> rvecs,
					     const std::vector<cv::Mat> tvecs,
					     const std::vector<std::vector<cv::Point2f> > imagePoints);
  public:
    cameraCalib();
    cameraCalib(cv::Mat cameraMatrix, cv::Mat distCoeffs, patternType pattern);
    
    void runCalibration(std::vector<cv::Mat>& images,
			const cv::Size boardSize,
			const float squareSize,
			const int debug
			);
    void undistortImage(std::vector<cv::Mat> images,
			std::vector<cv::Mat>& undistortedImages,
			int debug);
    void undistortImage(cv::Mat image,
			cv::Mat& undistortedImage,
			int debug);
  };
}
