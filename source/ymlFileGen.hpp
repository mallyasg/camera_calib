#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

int generateFileNameYML(int numImages){

  cv::FileStorage leftFS("left.yml", cv::FileStorage::WRITE);
  cv::FileStorage rightFS("right.yml", cv::FileStorage::WRITE);

  std::string leftFileName = "../data/left";
  std::string rightFileName = "../data/right";
  
  leftFS << "File Names" << "[";
  rightFS << "File Names" << "[";
  
  for (int i = 0; i < numImages; i++) {
    leftFS << leftFileName + std::to_string(i) + ".jpg";
    rightFS << rightFileName + std::to_string(i) + ".jpg";
  }

  leftFS << "]";
  rightFS << "]";

  leftFS.release();
  rightFS.release();
  
  return 0;
}
