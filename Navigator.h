#include <iostream>
#include <opencv2/opencv.hpp>

// using namespace std;
// using namespace cv;

class Navigation{
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
    
    Navigation(cv::Mat* m, int w, int h, cv::Point s, cv::Point g){
        std::cout << "\nNavigation\n";
        height = h;
        width = w;
        map = m;
        *map = cv::Mat(height, width, CV_8UC3, white);
        // drawGrid();
        startPoint = s;
        goalPoint = g;
        agent = startPoint;
        drawAgent(startPoint, betterBlue);
        drawAgent(goalPoint, goalColor);
    }

    bool moveStep(int stpX, int stpY, cv::Scalar leftOverColor = (204, 102, 0)){
        if (goalReached){
            return true;
        }

        cv::Point temp = agent;
        temp.x += stpX*inc;
        temp.y += stpY*inc;
        
        int stepCheck = checkStep(temp);

        if (stepCheck >= 0){
            circle(*map, agent, pointRadius, traveledColor, pointThickness);
            agent = temp;
            circle(*map, agent, pointRadius, agentColor, pointThickness);
            if (stepCheck == 1){
                goalReached = true;
                circle(*map, agent, pointRadius, reachedGoalColor, pointThickness);
            }
            return true;
        }
        else{
            return false;
        }
    }

    void drawAgent(cv::Point p, cv::Scalar color){
        circle(*map, p, pointRadius, color, pointThickness);
    }

    void drawWalls(cv::Point pt1, cv::Point pt2){
        line(*map, pt1, pt2, black, 15);
    }
    
    void drawGrid(cv::Scalar color = {128, 128, 128}){
        for (int i = 0; i < width; i += inc){
            line(*map, cv::Point{i,0}, cv::Point{i,height}, color, 1);
        }
        for (int i = 0; i < height; i += inc){
            line(*map, cv::Point{0,i}, cv::Point{width,i}, color, 1);
        }
    }

    int checkStep(cv::Point ag){
        // check map boundery
        if (ag.x > width || ag.x < 0 || ag.y > height || ag.y < 0){                 
            return -1;
        }
        // check its not a wall
        for (int dy = int(-inc / 2); dy <= int(inc / 2); dy++) {
            for (int dx = int(-inc / 2); dx <= int(inc / 2); dx++) {
                int nx = ag.x + dx;
                int ny = ag.y + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height){
                    continue;
                }
                cv::Vec3b& pixel = (*map).at<cv::Vec3b>(ny, nx);
                if (pixel[0] == black[0] && pixel[1] == black[1] && pixel[2] == black[2]){
                    return -1;
                }
            }
        }
        // got to goal
        if (ag.x == goalPoint.x && ag.y == goalPoint.y){                            
            return 1;
        }
        return 0;
    }
};