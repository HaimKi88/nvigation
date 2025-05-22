#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class NavigationSim{
public:
    cv::Scalar white{255, 255, 255};
    cv::Scalar black{0, 0, 0};
    cv::Scalar red{0, 51, 153};
    cv::Scalar blue{204, 102, 0};
    cv::Scalar betterBlue{170, 30, 0};
    cv::Scalar green{0, 128, 0};
    cv::Scalar brown{19,69,139};
    
    int height;
    int width;
    int inc = 10;
    
    cv::Mat* map;
    cv::Point startPoint;
    cv::Point goalPoint;
    cv::Point agent;
    bool goalReached = false;
    
    int pointRadius = 5, pointThickness = 5;
    cv::Scalar agentColor = black;
    cv::Scalar goalColor = green; 
    cv::Scalar traveledColor = blue;
    cv::Scalar reachedGoalColor = green;
    
    NavigationSim(cv::Mat* m, int w, int h, cv::Point s, cv::Point g);
    bool moveStep(int stpX, int stpY, cv::Scalar leftOverColor);
    void drawAgent(cv::Point p, cv::Scalar color);
    void drawWalls(cv::Point pt1, cv::Point pt2);   
    void drawGrid(cv::Scalar color);
    int checkStep(cv::Point ag);
};