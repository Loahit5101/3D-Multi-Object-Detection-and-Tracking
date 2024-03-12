#include "camera_detector.h"

CameraObjectDetector::CameraObjectDetector(std::string modelWeights, std::string modelConfig, std::string classesFilename, float confThresh, float nmsThresh)
    : _inputWidth(416), _inputHeight(416), _confidenceThresh(confThresh), _nonMaxSuppressionThresh(nmsThresh)
{
    // Load the network
    _detector = std::make_unique<cv::dnn::Net>(cv::dnn::readNetFromDarknet(modelConfig, modelWeights));
    _detector->setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    _detector->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // Load classes
    loadClasses(classesFilename);
}

void CameraObjectDetector::detectObject(cv::Mat &frame, std::vector<cv::Mat> &info)
{
    // Create a 4D blob from the frame
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(_inputWidth, _inputHeight), cv::Scalar(0, 0, 0), true, false);

    // Set the input blob
    _detector->setInput(blob);

    // Forward pass
    std::vector<cv::Mat> outs;
    _detector->forward(outs, getOutputsNames(*_detector));

    // Scan through all the bounding boxes and keep only the ones with high confidence
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network
        float *data = (float *)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > _confidenceThresh) // Filter the weak detections
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }

    // Perform non-maximum suppression to eliminate overlapping bounding boxes
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, _confidenceThresh, _nonMaxSuppressionThresh, indices);

    // Save information for each detection
    info.clear();
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        int classId = classIds[idx];
        float confidence = confidences[idx];

        info.push_back(cv::Mat(1, 7, CV_32F));
        cv::Mat &detection = info.back();
        detection.at<float>(0, 0) = box.x;
        detection.at<float>(0, 1) = box.y;
        detection.at<float>(0, 2) = box.x + box.width;
        detection.at<float>(0, 3) = box.y + box.height;
        detection.at<float>(0, 4) = confidence;
        detection.at<float>(0, 5) = classId;
        detection.at<float>(0, 6) = static_cast<float>(idx);
    }
}

void CameraObjectDetector::drawDetections(cv::Mat &image, std::vector<cv::Mat> &info)
{
    for (const auto &detection : info)
    {
        float confidence = detection.at<float>(0, 4);
        int classId = static_cast<int>(detection.at<float>(0, 5));
        int boxId = static_cast<int>(detection.at<float>(0, 6));
        int left = static_cast<int>(detection.at<float>(0, 0));
        int top = static_cast<int>(detection.at<float>(0, 1));
        int right = static_cast<int>(detection.at<float>(0, 2));
        int bottom = static_cast<int>(detection.at<float>(0, 3));

        // Draw the bounding box
        cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);

        // Put label with confidence score
        std::string label = cv::format("%s: %.2f", _classes[classId].c_str(), confidence);
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::putText(image, label, cv::Point(left, top - labelSize.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}


std::vector<std::string> CameraObjectDetector::getOutputsNames(const cv::dnn::Net &net)
{
    static std::vector<std::string> names;
    if (names.empty())
    {
        // Get the indices of the output layers
        std::vector<int> outLayers = net.getUnconnectedOutLayers();

        // Get the names of the output layers
        std::vector<std::string> layersNames = net.getLayerNames();
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        {
            names[i] = layersNames[outLayers[i] - 1];
        }
    }
    return names;
}

void CameraObjectDetector::loadClasses(std::string classesFilename)
{
    std::fstream file(classesFilename);
    std::string line;
    while (std::getline(file, line))
    {
        _classes.push_back(line);
    }
    std::cout << "Loaded " << _classes.size() << " Class Names" << std::endl;
}