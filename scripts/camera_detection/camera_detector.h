#ifndef CAMERA_OBJECT_DETECTOR_H
#define CAMERA_OBJECT_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>

class CameraObjectDetector
{
private:
    int _inputWidth;
    int _inputHeight;
    float _confidenceThresh;
    float _nonMaxSuppressionThresh;
    std::unique_ptr<cv::dnn::Net> _detector;
    std::vector<std::string> _classes;

public:
    CameraObjectDetector(std::string modelWeights, std::string modelConfig, std::string classesFilename, float confThresh, float nmsThresh);

    void detectObject(cv::Mat &frame, std::vector<cv::Mat> &info);

    void drawDetections(cv::Mat &image, std::vector<cv::Mat> &info);

private:
    std::vector<std::string> getOutputsNames(const cv::dnn::Net &net);

    void loadClasses(std::string classesFilename);
};

#endif /* CAMERA_OBJECT_DETECTOR_H */
