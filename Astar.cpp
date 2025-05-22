#include "AStar.h"

AStar::AStar(cv::Mat* m, int w, int h, cv::Point s, cv::Point g) : NavigationSim(m, w, h, s, g){
        std::cout << "-- A* navigation --\n" << std::endl;
}
    
std::pair<int, Node_> AStar::lowestFcost(const std::vector<Node_>& vec){
    if (vec.empty()) {
        std::cout << "Cannot get lowest fcost from empty vector." << std::endl;
    }

    auto lowest = min_element(vec.begin(), vec.end(), 
                    [](const Node_& a, const Node_& b){
                        return a.fcost < b.fcost;
                    });
    int index = distance(vec.begin(), lowest);
    return {index, *lowest};
}

bool AStar::isInVector(const std::vector<Node_>& vec, Node_ temp){
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

std::vector<Node_> AStar::findNeighbours(Node_& current){
    std::vector<Node_> neighbours;
    Node_ tmpNode;
    cv::Point nxt;
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

int AStar::calcHcost(Node_ n, int heuristics = 1){
    if (heuristics == 1){       // Manhattan distance
        return abs(n.i - goalPoint.x) + abs(n.j - goalPoint.y);
    }
    else if (heuristics == 2){  // Euclidean distance
        return int(sqrt((n.i - goalPoint.x)^2 + (n.j - goalPoint.y)^2));
    }
    return 0;
}

void AStar::reconstructPath(Node_ current){
    Node_ tmpNode = current;
    cv::Point tmpPoint;
    std::cout << "reconstruct path" << std::endl;
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

void AStar::initAlgo(){
    current.gcost = 0;
    open.push_back(current);
}

int AStar::runAlgo(int k){

    auto [index, current] = lowestFcost(open); // node in open with lowest fcost
    
    open.erase(open.begin() + index);          // remove current from open
    
    closed.push_back(current);                 // add current to closed
    
    if (current.i == goalPoint.x && current.j == goalPoint.y){
        std::cout << "reached goal node!\n" << std::endl;
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
