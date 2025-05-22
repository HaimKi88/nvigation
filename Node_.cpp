#include "Node_.h"

Node_::Node_(){}

Node_::Node_(int ii, int jj) : i(ii), j(jj) {}

void Node_::printLocation(){
    std::cout << "Location: {" << i << ", " << j << "} " << std::endl;
}

void Node_::printInfo(){
    std::cout << "Location: {" << i << ", " << j << "}" << 
    ", hcost: " << hcost << 
    ", gcost: " << gcost << 
    ", fcost: " << fcost << 
    ", parent: {" << parent[0] << ", " << parent[1] << "}" << std::endl;
}