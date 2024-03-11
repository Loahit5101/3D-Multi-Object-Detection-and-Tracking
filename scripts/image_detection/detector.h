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

    inline void addObjectClassToImage(int classId, float conf, int left, int top, cv::Mat& image)
    {
      std::string label = std::to_string(conf);
      if (!_classes.empty())
	    { 
      	label = _classes[classId] + ":" + label;
    	}
      else
      {
        return;
      }
      
      //Add the label at the top of the bounding box
      int baseLine;
      cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseLine);
      top = std::max(top, labelSize.height);
      cv::rectangle(image, cv::Point(left, top), cv::Point(left+labelSize.width, top - std::min(top, labelSize.height)), cv::Scalar(255, 0, 255), -1);
      cv::putText(image, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255));
    }


};