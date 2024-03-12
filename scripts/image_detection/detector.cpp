#include "detector.h"

CameraDetector::CameraDetector(std::string modelWeights, std::string modelConfig, std::string classesFilename, float confThresh, float nmsThresh)
{    
    _confidenceThresh = confThresh;
    _nonMaxSuppressionThresh = nmsThresh;
    _detector = std::make_unique<cv::dnn::Net>(cv::dnn::readNetFromDarknet(modelConfig, modelWeights));
    _detector->setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    _detector->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    if(_detector->empty())
    {
        std::cout << "Model Load Error" << std::endl;
        exit(-1);
    }
    else
    {
        std::cout << "Model Loaded Successfully!" << std::endl;
    }
    
    loadClasses(classesFilename);

    // For better detection set 416
    _inputWidth = 320;
    _inputHeight = 320;

}

void CameraDetector::detectObject(cv::Mat &frame, std::vector<cv::Mat> &info)
{
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(_inputWidth, _inputHeight), cv::Scalar(0,0,0), true, false);
	
	_detector->setInput(blob);

	_detector->forward(info, _detector->getUnconnectedOutLayersNames());
}

void CameraDetector::drawDetections(cv::Mat &image, std::vector<cv::Mat> &detection_info){

        for (const auto& detection : detection_info) {

        float confidence = detection.at<float>(5); // Confidence score
        int left = static_cast<int>(detection.at<float>(0) * image.cols);
        int top = static_cast<int>(detection.at<float>(1) * image.rows);
        int right = static_cast<int>(detection.at<float>(2) * image.cols);
        int bottom = static_cast<int>(detection.at<float>(3) * image.rows);

        if (confidence > 0.5) { 
            cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);
        }

        cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);
    }
}
