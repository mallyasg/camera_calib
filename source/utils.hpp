#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

void readCalibrationImages(std::string YMLFileName,
			   std::vector<cv::Mat> &images,
			   const int debug);
