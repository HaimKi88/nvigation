#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

using namespace std;
using namespace cv;

class Node_{
    public:
        int i, j;
        array<int, 2> parent{-1,-1};
        int hcost = 0, gcost = 99999, fcost = 0;
        int gcostFromCrnt;

        Node_(){}

        Node_(int ii, int jj){
            i = ii;
            j = jj;
        }

        void printLocation(){
            cout << "Location: {" << i << ", " << j << "} " << endl;
        }

        void printInfo(){
            cout << "Location: {" << i << ", " << j << "}" << 
            ", hcost: " << hcost << 
            ", gcost: " << gcost << 
            ", fcost: " << fcost << 
            ", parent: {" << parent[0] << ", " << parent[1] << "}" << endl;
        }
    };

class Navigation{
public:
    Scalar white{255, 255, 255};
    Scalar black{0, 0, 0};
    Scalar red{0, 51, 153};
    Scalar blue{204, 102, 0};
    Scalar betterBlue{170, 30, 0};
    Scalar green{0, 128, 0};
    Scalar brown{19,69,139};
    
    int height;
    int width;
    int inc = 10;
    
    Mat* map;
    Point startPoint;
    Point goalPoint;
    Point agent;
    bool goalReached = false;
    
    int pointRadius = 5, pointThickness = 5;
    Scalar agentColor = black;
    Scalar goalColor = green; 
    Scalar traveledColor = blue;
    Scalar reachedGoalColor = green;
    
    Navigation(Mat* m, int w, int h, Point s, Point g){
        cout << "\nNavigation\n";
        height = h;
        width = w;
        map = m;
        *map = Mat(height, width, CV_8UC3, white);
        // drawGrid();
        startPoint = s;
        goalPoint = g;
        agent = startPoint;
        drawAgent(startPoint, betterBlue);
        drawAgent(goalPoint, goalColor);
    }

    bool moveStep(int stpX, int stpY, Scalar leftOverColor = (204, 102, 0)){
        if (goalReached){
            return true;
        }

        Point temp = agent;
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

    void drawAgent(Point p, Scalar color){
        circle(*map, p, pointRadius, color, pointThickness);
    }

    void drawWalls(Point pt1, Point pt2){
        line(*map, pt1, pt2, black, 15);
    }
    
    void drawGrid(Scalar color = {128, 128, 128}){
        for (int i = 0; i < width; i += inc){
            line(*map, Point{i,0}, Point{i,height}, color, 1);
        }
        for (int i = 0; i < height; i += inc){
            line(*map, Point{0,i}, Point{width,i}, color, 1);
        }
    }

    int checkStep(Point ag){
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
                Vec3b& pixel = (*map).at<Vec3b>(ny, nx);
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

    void targetReachedMsg(){
        Point loc(width- width/3, height/10);
        putText(*map, "Goal reached!", loc, FONT_HERSHEY_SIMPLEX, 1, red, 1, LINE_AA);
    }
};

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
    Mat robotImg = imread("C:\\Users\\Haim\\Documents\\development\\robot.png");
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