#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>


class AStar : public NavigationSim {
public:
    std::vector<Node_> open;
    std::vector<Node_> closed;
    std::vector<Node_> bestPath;
    // Run algo params:
    std::vector<Node_> neighbours;
    Node_ current{startPoint.x, startPoint.y};

    AStar(cv::Mat* m, int w, int h, cv::Point s, cv::Point g);
        
    std::pair<int, Node_> lowestFcost(const std::vector<Node_>& vec);

    bool isInVector(const std::vector<Node_>& vec, Node_ temp);

    std::vector<Node_> findNeighbours(Node_& current);

    int calcHcost(Node_ n, int heuristics);

    void reconstructPath(Node_ current);

    void initAlgo();

    int runAlgo(int k);

};
