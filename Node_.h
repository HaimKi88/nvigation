#pragma once
#include <iostream>
#include <array>

class Node_{
    public:
        Node_();
        Node_(int ii, int jj);
        int i, j;
        std::array<int, 2> parent{-1,-1};
        int hcost = 0, gcost = 99999, fcost = 0;
        int gcostFromCrnt;
        void printLocation();
        void printInfo();
       
};
