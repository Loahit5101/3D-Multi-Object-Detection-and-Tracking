#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>

class CameraDetector
{
  private:
    
    int _inputWidth;
    int _inputHeight;

  public:

    float _confidenceThresh;
    float _nonMaxSuppressionThresh;

    std::unique_ptr<cv::dnn::Net> _detector;
    std::vector<std::string> _classes;

    CameraDetector(std::string modelWeights, std::string modelConfig, std::string classesFilename, float confThresh, float nmsThresh);

    void detectObject(cv::Mat &frame, std::vector<cv::Mat> &info);

    void drawDetections(cv::Mat &image, std::vector<cv::Mat> &info);

    inline void loadClasses(std::string classesFilename)
    {
      std::fstream file(classesFilename);
      std::string line;
      while(std::getline(file, line))
      {
        _classes.push_back(line);
      }
      std::cout << "Loaded " << _classes.size() <<" Class Names" << std::endl;
    }


};