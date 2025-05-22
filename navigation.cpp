#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <array>
#include <thread>

#include "Node_.h"
#include "Navigator.h"

using namespace std;
using namespace cv;

class AStar : public Navigation {
public:
    AStar(Mat* m, int w, int h, Point s, Point g) : Navigation(m, w, h, s, g){
        cout << "-- A* navigation --\n";
    }

    vector<Node_> open;
    vector<Node_> closed;
    vector<Node_> bestPath;
    
    // Run algo params:
    vector<Node_> neighbours;
    Node_ current{startPoint.x, startPoint.y};
    
    pair<int, Node_> lowestFcost(const vector<Node_>& vec){
        if (vec.empty()) {
            throw runtime_error("Cannot get lowest fcost from empty vector.");
        }

        auto lowest = min_element(vec.begin(), vec.end(), 
                        [](const Node_& a, const Node_& b){
                            return a.fcost < b.fcost;
                        });
        int index = distance(vec.begin(), lowest);
        return {index, *lowest};
    }

    bool isInVector(const vector<Node_>& vec, Node_ temp){
        if (vec.empty()) {
            return false;
        }

        for (Node_ nd : vec){
            if (nd.i == temp.i && nd.j == temp.j){
                return true;
            }
        }
        return false;
    }

    vector<Node_> findNeighbours(Node_& current){
        vector<Node_> neighbours;
        Node_ tmpNode;
        Point nxt;
        int step;

        for (int i = -1; i < 2; ++i){
            for (int j = -1; j < 2; ++j){
                if (i == 0 && j == 0){
                    continue;
                }
                nxt.x = current.i + i*inc;
                nxt.y = current.j + j*inc;
                
                step = checkStep(nxt);                      // check if step is accessible
                if (step == 0 || step == 1){
                    tmpNode = Node_{nxt.x, nxt.y};
                    if (isInVector(closed, tmpNode)){              // check if node in closed 
                        continue;
                    }
                    else{
                        circle(*map, nxt, pointRadius, red, pointThickness);
                        if (i != 0 && j != 0){tmpNode.gcostFromCrnt = 14;}  // set gcost
                        else {tmpNode.gcostFromCrnt = 10;}
                        neighbours.push_back(tmpNode);
                    }
                }
            }
        }
        return neighbours;
    }

    int calcHcost(Node_ n, int heuristics = 1){
        if (heuristics == 1){       // Manhattan distance
            return abs(n.i - goalPoint.x) + abs(n.j - goalPoint.y);
        }
        else if (heuristics == 2){  // Euclidean distance
            return int(sqrt((n.i - goalPoint.x)^2 + (n.j - goalPoint.y)^2));
        }
        return 0;
    }

    void reconstructPath(Node_ current){
        Node_ tmpNode = current;
        Point tmpPoint;
        cout << "reconstruct path" << endl;
        bestPath.push_back(current);
        int k = 0;
        while(true){
            for (Node_ nd : closed){
                if (tmpNode.parent[0] == -1 && tmpNode.parent[1] == -1){        // reached start point
                    drawAgent(goalPoint, goalColor);
                    return;
                }
                else if (tmpNode.parent[0] == nd.i && tmpNode.parent[1] == nd.j){
                    tmpNode = nd;
                }
            }
            
            bestPath.push_back(tmpNode);
            tmpPoint.x = tmpNode.i; 
            tmpPoint.y = tmpNode.j; 
            
            drawAgent(tmpPoint, traveledColor);
        }
    }

    void initAlgo(){
        current.gcost = 0;
        open.push_back(current);
    }

    int runAlgo(int k){
    
        auto [index, current] = lowestFcost(open); // node in open with lowest fcost
        
        open.erase(open.begin() + index);          // remove current from open
        
        closed.push_back(current);                 // add current to closed
        
        if (current.i == goalPoint.x && current.j == goalPoint.y){
            cout << "reached goal node!\n" << endl;
            reconstructPath(current);
            return 0;
        }
        
        neighbours = findNeighbours(current);      // check neightbours and if in closed vector

        for (Node_ n : neighbours){
            int tmpGcost = current.gcost + n.gcostFromCrnt;
            if (tmpGcost < n.gcost){
                n.parent = {current.i, current.j};
                n.gcost = tmpGcost;
                n.hcost = calcHcost(n);
                n.fcost = n.gcost + n.hcost;
                if (isInVector(open, n) == false){
                    open.push_back(n);
                }
            }
        }
        drawAgent(startPoint, betterBlue);
        
        if (k == 1000 ){  // prevent infint loop
            return 0;
        }
        else{
            return 1;
        }
    }

};

int main(){
    int height = 900, width = 1100;
    // Point startPoint = {15, 15};
    // Point goalPoint = {startPoint.x+60*10, startPoint.y+80*10};   
    Point startPoint = {15, 200};
    Point goalPoint = {startPoint.x+100*10, startPoint.y+40*10};   
    
    Mat map;
    Point pt1,pt2;
    int k = 0;
    int result = 1;
    AStar nav(&map, width, height, startPoint, goalPoint);
    
    /////////// obstacles: /////////////////
    for (int spaces = 100; spaces <= 700; spaces += 100 ){
        nav.drawWalls(Point{spaces,320}, Point{spaces+50,320});
    }
    putText(map, "Pathfinding", Point(100, 300), HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, LINE_AA);
    putText(map, "Software", Point(100, 430), HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, LINE_AA);
    putText(map, "Robotics", Point(100, 560), HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, LINE_AA);
    putText(map, "C++ / Python", Point(100, 690), HersheyFonts::FONT_HERSHEY_PLAIN, 7, nav.black, 10, LINE_AA);
    // robot picture
    Mat robotImg = imread("C:\\Users\\Haim\\Documents\\development\\navigation\\robot.png");
    double scale = 0.5; 
    Mat resizedRobot;
    resize(robotImg, resizedRobot, Size(), scale, scale);
    Rect roi(Point(800,10), resizedRobot.size());
    resizedRobot.copyTo(map(roi));
    ///////////////////////////////////////
    nav.initAlgo();
    
    while (true){
        imshow("map", map);
        
        if (result == 1){
            result = nav.runAlgo(k); 
            k++;
        }

        if (cv::waitKey(1) == 'q') {
            break;        
        }
        this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return 0;
}

