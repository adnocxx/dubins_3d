#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <cstring>
using namespace std;
int main(){

struct all_points{  //存储所有路径点的时刻和坐标
       vector<float>x;
       vector<float>y;
       vector<float>z;
       vector<float>t;
};

all_points A;
A.x.push_back(1);
A.x.push_back(2);
A.x.push_back(3);
cout<<A.x[0]<<endl;
cout<<A.x[1]<<endl;
cout<<A.x[2]<<endl;
cout<<A.x.size()<<endl;



}
