#include <iostream>
#include <opencv2/opencv.hpp>

class Record{
public:
    bool recordEnabled;
    // Define video parameters
    std::string outputVideo;
    int fps = 30;
    int duration_sec = 5;
    int total_frames = fps * duration_sec;

    // Define the codec and create VideoWriter
    cv::VideoWriter writer;
    cv::Size frameSize;
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    
    Record(cv::Mat image,std::string outputV, bool rec){
        outputVideo = outputV;
        recordEnabled = rec;
        frameSize = {image.cols, image.rows};
    }

    void init() {
        if (!recordEnabled) return;
        writer.open(outputVideo, codec, fps, frameSize, true);
        if (!writer.isOpened()) {
            std::cerr << "Error: Could not open video writer!" << std::endl;
        }
    }

    void record(const cv::Mat& currentImage) {
        if (!recordEnabled) return;
        if (writer.isOpened()) {
            writer.write(currentImage);
        }
    }

    void endRecord() {
        if (!recordEnabled) return;
        if (writer.isOpened()) {
            writer.release();
            std::cout << "Video saved to: " << outputVideo << std::endl;
        }
    }
};
