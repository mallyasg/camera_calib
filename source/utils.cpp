#include "utils.hpp"

void readCalibrationImages(std::string ymlFileName,
			   std::vector<cv::Mat> &images) {
  
  cv::FileStorage YMLFS(ymlFileName, cv::FileStorage::READ);
  std::vector<std::string> imageFileNames;
  
  cv::FileNodeIterator it = YMLFS["File Names"].begin();
  cv::FileNodeIterator it_end = YMLFS["File Names"].end();

  for ( ; it != it_end; ++it) {
    assert ((*it).isString());
    imageFileNames.push_back((std::string)(*it));
  }
  
  // Iterate through the list containing image filenames
  for (auto fileName : imageFileNames) {
    cv::Mat tempImage = cv::imread(fileName, cv::IMREAD_GRAYSCALE);
    assert(tempImage.data);
    images.push_back(tempImage);
  }
  
# ifdef DEBUG
  for (unsigned int i = 0; i < images.size(); ++i) {
    cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
    cv::imshow("Display Image", images[i]);
    cv::waitKey(0);
  }
# endif
}
