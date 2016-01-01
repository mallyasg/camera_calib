#include "utils.hpp"
#include "cameraCalib.hpp"

int main(int argc, char **argv){
  
  std::string ymlFileName = argv[1];
  int boardWidth          = std::stoi(argv[2]);
  int boardHeight         = std::stoi(argv[3]);
  float squareSize        = std::stof(argv[4]);
  int debug               = std::stoi(argv[5]);
  
  std::vector<cv::Mat> images;
  std::vector<cv::Mat> undistortedImages;
  
  cv::Size boardSize = cv::Size(boardWidth, boardHeight);
  
  if (ymlFileName.compare("0")){
    
    squareSize = 3.0;
    // Read the yml file that contains the images from the left camera for
    // calibration
    readCalibrationImages(ymlFileName, images, debug);
    // Calibrate the camera
    gsm::cameraCalib calibHandler;
    calibHandler.runCalibration(images, boardSize, squareSize, debug);
    calibHandler.undistortImage(images, undistortedImages, debug);
  } else {
    
  }
  return 0;
}

