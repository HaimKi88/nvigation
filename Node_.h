#include <iostream>
#include <array>

class Node_{
    public:
        int i, j;
        std::array<int, 2> parent{-1,-1};
        int hcost = 0, gcost = 99999, fcost = 0;
        int gcostFromCrnt;

        Node_(){}

        Node_(int ii, int jj){
            i = ii;
            j = jj;
        }

        void printLocation(){
            std::cout << "Location: {" << i << ", " << j << "} " << std::endl;
        }

        void printInfo(){
            std::cout << "Location: {" << i << ", " << j << "}" << 
            ", hcost: " << hcost << 
            ", gcost: " << gcost << 
            ", fcost: " << fcost << 
            ", parent: {" << parent[0] << ", " << parent[1] << "}" << std::endl;
        }
    };
