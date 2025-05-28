#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <array>
#include <thread>

#include "Node_.cpp"
#include "NavigationSim.cpp"
#include "AStar.cpp"
#include "Record.cpp"

cv::Mat map;
int height = 950, width = 1689;
int k = 0;
int result = 1;
bool firstRun = true;
cv::Point startPoint = {15, 200}; //{15, 15};
cv::Point goalPoint = {startPoint.x+100*10, startPoint.y+40*10};  // {startPoint.x+60*10, startPoint.y+80*10};
AStar nav(&map, width, height, startPoint, goalPoint);

std::string outputVideo = "C:\\Users\\Haim\\Documents\\development\\navigation\\pathFinding.avi";

void obstacles(){
    for (int spaces = 100; spaces <= 700; spaces += 100 ){
        nav.drawWalls(cv::Point{spaces,320}, cv::Point{spaces+50,320});
    }
    putText(map, "Pathfinding", cv::Point(100, 300), cv::HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, cv::LINE_AA);
    putText(map, "Software", cv::Point(100, 430), cv::HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, cv::LINE_AA);
    putText(map, "Robotics", cv::Point(100, 560), cv::HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, cv::LINE_AA);
    putText(map, "C++ / Python", cv::Point(100, 690), cv::HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, cv::LINE_AA);
    // robot picture
    cv::Mat robotImg = cv::imread("C:\\Users\\Haim\\Documents\\development\\navigation\\robot.png");
    double scale = 0.5; 
    cv::Mat resizedRobot;
    resize(robotImg, resizedRobot, cv::Size(), scale, scale);
    cv::Rect roi(cv::Point(800,10), resizedRobot.size());
    resizedRobot.copyTo(map(roi));
}

int main(){
    Record rec(map, outputVideo, true);
    obstacles();

    nav.initAlgo();
    
    rec.init();
    imshow("map", map);     // wait before starting
    cv::waitKey(500);
    
    while (true){
        imshow("map", map);
        rec.record(map);

        if (result == 1){
            result = nav.runAlgo(k); 
            k++;
        }

        if (cv::waitKey(1) == 'q') {
            rec.endRecord();
            break;        
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return 0;
}

