#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <condition_variable>

#include "Node_.cpp"
#include "NavigationSim.cpp"
#include "AStar.cpp"
#include "Record.cpp"

cv::Mat map;
int height = 950, width = 1689;
bool firstRun = true;

std::mutex mtx;
std::condition_variable agent_cv;
bool sim_tick = false;
std::atomic<bool> done = false;

cv::Point startPoint = {15, 200}; //{15, 15};
cv::Point goalPoint = {startPoint.x+100*10, startPoint.y+60*10};  // {startPoint.x+60*10, startPoint.y+80*10};
cv::Point startPoint2 = {15*5, 200*2}; //{15, 15};
cv::Point goalPoint2 = {startPoint.x+100*10, startPoint.y+40*10};  // {startPoint.x+60*10, startPoint.y+80*10};

AStar nav(&map, width, height, startPoint, goalPoint);
AStar nav2(&map, width, height, startPoint2, goalPoint2);

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

void agent(int agentId, AStar* n){
    int k = 0;
    bool result = 1;
    std::cout << "starting thread  " << agentId << std::endl;
    n->initAlgo();

    std::unique_lock<std::mutex> lock(mtx);
    while(true){
        agent_cv.wait(lock, [] { return sim_tick || done; });
        if (done || result == 0) break;

        result = n->runAlgo(k);
        k++;
        if (result == 0){
            break;
        }
        sim_tick = false;
    }
    std::cout << "exiting agent thread " << agentId << std::endl;
}

int main(){
    Record rec(map, outputVideo, false);
    std::thread a1(agent, 1, &nav);
    std::thread a2(agent, 2, &nav2);

    obstacles();
    
    rec.init();
    imshow("map", map);     // wait before starting
    cv::waitKey(500);
    

    while (true){
        imshow("map", map);
        rec.record(map);

        {
            std::lock_guard<std::mutex> lock(mtx);
            sim_tick = true;
        }
        agent_cv.notify_all();

        if (cv::waitKey(1) == 'q') {
            rec.endRecord();
            done = true;
            agent_cv.notify_all();
            break;        
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    a1.join();
    a2.join();

    return 0;
}

