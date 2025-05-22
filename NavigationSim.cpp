#include "NavigationSim.h"

NavigationSim::NavigationSim(cv::Mat* m, int w, int h, cv::Point s, cv::Point g){
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

bool NavigationSim::moveStep(int stpX, int stpY, cv::Scalar leftOverColor = (204, 102, 0)){
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

void NavigationSim::drawAgent(cv::Point p, cv::Scalar color){
    circle(*map, p, pointRadius, color, pointThickness);
}

void NavigationSim::drawWalls(cv::Point pt1, cv::Point pt2){
    line(*map, pt1, pt2, black, 15);
}

void NavigationSim::drawGrid(cv::Scalar color = {128, 128, 128}){
    for (int i = 0; i < width; i += inc){
        line(*map, cv::Point{i,0}, cv::Point{i,height}, color, 1);
    }
    for (int i = 0; i < height; i += inc){
        line(*map, cv::Point{0,i}, cv::Point{width,i}, color, 1);
    }
}

int NavigationSim::checkStep(cv::Point ag){
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