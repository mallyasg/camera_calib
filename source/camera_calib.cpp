#include "utils.hpp"
#include "cameraCalib.hpp"

int main(int argc, char **argv){
  
  std::string ymlFileName = argv[1];
  std::vector<cv::Mat> images;
  // Read the yml file that contains the images from the left camera for
  // calibration
  readCalibrationImages(ymlFileName, images);
  // Calibrate the camera
  gsm::cameraCalib calibHandler;
  std::vector<std::vector<cv::Point2f> > imagePoints;
  
  calibHandler.findPattern(images, cv::Size(7, 6), imagePoints);
  return 0;
}

