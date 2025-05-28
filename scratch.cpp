#include <iostream>
#include <memory>
// #include <array>
#include <vector>

using namespace std;

void printArr(int* parr){
    for (int i = 0; i < 5; i++){
        cout << *parr + i << endl;
    }
    cout << endl;
    for (int i = 0; i < 5; i++){
        cout << *(parr + i) << endl;
    }
}

int main() {
    int i = 4;
    int arr[5];
    int* parr = arr;
    int* p = &i;
    vector<int> vec;
    vector<int>* pvec = &vec;

    for (int i = 0; i < 5; i++){
        vec.push_back(i);
    }
    pvec->at(3) = 15; 
    for (int i = 0; i < 5; i++){
        cout << pvec->at(i) << endl;
    }
    return 0;
}
