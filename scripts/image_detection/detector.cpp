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
    _inputWidth = 1241;
    _inputHeight = 376;

}

void CameraDetector::detectObject(cv::Mat &frame, std::vector<cv::Mat> &info)
{
    // Create blob from the Image
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(_inputWidth, _inputHeight), cv::Scalar(0,0,0), true, false);
	
    // Sets the input to the network
	_detector->setInput(blob);

	// Runs the forward pass through the network to get output of the output layers
	_detector->forward(info, _detector->getUnconnectedOutLayersNames());
}
