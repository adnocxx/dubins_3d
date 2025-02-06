//无终值条件的三维dubins路径规划 两个无人机规划后检测碰撞 ---adnocxx
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <cstring>

using namespace std;

#define PI 3.1415926535897932384
struct UAV{
     float x[2],y[2],z[2],R_xy,v,theat,fai;
     //x,y,z为无人机的三维坐标,包括开始和结尾
     //R_xy为平面最小转弯半径，由偏航角决定
     //v为无人机速度，假设为匀速运动
     //theat为初始偏航角，假设未有俯仰角和滚转角
     //fai是最大俯仰角
};

class dubins_cheak{
     private:
     struct circle{
        float bigin_point[2];//开始点的 坐标
        float R;//圆的半径
        float end_point[2];//结束切点的 坐标
        float circle_point[2];//圆心坐标
        string type;
     }; 
     struct all_points{  //存储所有路径点的时刻和坐标
       vector<float>x;
       vector<float>y;
       vector<float>z;
       vector<float>t;
     };
      UAV coordinate(UAV A){  // 输入无人机A的信息，进行坐标转换，（连接出发点和起始点，正方向为出发点到起始点，转化坐标和角度）
         float k=atan2((A.y[1]-A.y[0]),(A.x[1]-A.x[0]))*180/PI;//计算两点直线与x轴连线的夹角
         float l=sqrt((A.x[0]-A.x[1])*(A.x[0]-A.x[1])+(A.y[0]-A.y[1])*(A.y[0]-A.y[1]));//求出两点之间的距离
         if(A.x[0]==A.x[1] && A.y[0]<A.y[1]){k=-90;} //限制四个特殊情况
         if(A.x[0]==A.x[1] && A.y[0]>A.y[1]){k=90;} 
         if(A.y[0]==A.y[1] && A.x[0]<A.x[1] ){k=180;}
         if(A.y[0]==A.y[1] && A.x[0]<A.x[1] ){k=0;}
         A.x[0]=0;
         A.y[0]=0;
         A.x[1]=l;
         A.y[1]=0;
         A.theat=A.theat-k;
   
         //控制在0-360度范围内
         for(int i=1;;i++){
             if(A.theat>=360){
                A.theat=A.theat-360;
                }
             else if(A.theat<0){
                A.theat=A.theat+360;
                }
             else {
               break;
                }
         };
           cout<<endl;
           cout<<"出发点的结果坐标:["<<A.x[0]<<","<<A.y[0]<<"]"<<endl;
           cout<<"结束点的结果坐标:["<<A.x[1]<<","<<A.y[1]<<"]"<<endl;
           cout<<"变换后偏航角为:["<<A.theat<<"°]"<<endl; 
           cout<<endl;
     
          //获得新的坐标和初始角；
         return A;
         //利用新的坐标和初始角计算
         //Theta_1+180-k;
     };

     circle path_circle_shun(UAV A){  //输入转化后无人机A的坐标，输出顺时针轨迹圆的信息
      
      //theta-90°对应的圆是顺时针圆
      circle B;  //存储两个圆，一个为正（顺时针），一个为逆（逆时针）
      B.R=A.R_xy;
      B.bigin_point[0]=0;
      B.bigin_point[1]=0;
      B.type="+";
      B.circle_point[0]=A.R_xy*cos((A.theat-90)*PI/180);  //顺时针圆的横坐标
      B.circle_point[1]=A.R_xy*sin((A.theat-90)*PI/180);  //顺时针圆的纵坐标
      float distance=sqrt((A.x[1]-B.circle_point[0])*(A.x[1]-B.circle_point[0])+(A.y[1]-B.circle_point[1])*(A.y[1]-B.circle_point[1])); //求出目标点和圆心的距离 distance
      float alpha=acos(A.R_xy/distance);                                          //求出切线三角形以圆心角为顶点的角度 alpha
      float beta=atan2(A.y[1]-B.circle_point[1],A.x[1]-B.circle_point[0]); //求出目标点和圆心连线与x轴的夹角 beta
      B.end_point[0]=B.circle_point[0]+A.R_xy*cos(beta+alpha);
      B.end_point[1]=B.circle_point[1]+A.R_xy*sin(beta+alpha);

      //打印输出信息
      cout<<endl;
      cout<<"顺时针圆的圆心坐标为：["<<B.circle_point[0]<<","<<B.circle_point[1]<<"]"<<endl;
      cout<<"出发点坐标为：["<<B.bigin_point[0]<<","<<B.bigin_point[1]<<"]"<<endl;
      cout<<"切点坐标为：["<< B.end_point[0]<<","<<B.end_point[1]<<"]"<<endl;
      cout<<"半径为：["<< A.R_xy<<"]"<<endl;
      cout<<endl;
      return B;
     }

     circle path_circle_ni(UAV A){    //输入转化后无人机A的坐标，输出逆时针轨迹圆的信息
      //theta+90°对应的圆是逆时针圆 
      circle B;
      B.R=A.R_xy;
      B.bigin_point[0]=0;
      B.bigin_point[1]=0;
      B.type="-";
      B.circle_point[0]=A.R_xy*cos((A.theat+90)*PI/180);  //逆时针圆心的横坐标
      B.circle_point[1]=A.R_xy*sin((A.theat+90)*PI/180);  //逆时针圆心的纵坐标
      float distance=sqrt((A.x[1]-B.circle_point[0])*(A.x[1]-B.circle_point[0])+(A.y[1]-B.circle_point[1])*(A.y[1]-B.circle_point[1])); //求出目标点和圆心的距离 distance
      float alpha=acos(A.R_xy/distance);                                          //求出切线三角形以圆心角为顶点的角度 alpha
      float beta=atan2(A.y[1]-B.circle_point[1],A.x[1]-B.circle_point[0]); //求出目标点和圆心连线与x轴的夹角 beta
      B.end_point[0]=B.circle_point[0]+A.R_xy*cos(beta-alpha);
      B.end_point[1]=B.circle_point[1]+A.R_xy*sin(beta-alpha);
      
      cout<<endl;
      cout<<"逆时针圆的圆心坐标为：["<<B.circle_point[0]<<","<<B.circle_point[1]<<"]"<<endl;
      cout<<"出发点坐标为：["<<B.bigin_point[0]<<","<<B.bigin_point[1]<<"]"<<endl;
      cout<<"切点坐标为：["<< B.end_point[0]<<","<<B.end_point[1]<<"]"<<endl;
      cout<<"半径为：["<< A.R_xy<<"]"<<endl;
      cout<<endl;
      return B;
      };
     
     float get_circle_distance(circle A){   //输入圆路径的信息，输出路径的长度
      
       float o_begin=atan2(A.bigin_point[1]-A.circle_point[1],A.bigin_point[0]-A.circle_point[0]) ;//圆心指向开始点的向量角度
       float o_end=atan2(A.end_point[1]-A.circle_point[1],A.end_point[0]-A.circle_point[0]) ;//圆心指向结束点的向量角度
         //标准化角度
       if(o_begin>=2*PI){o_begin=o_begin-2*PI;}
       else if(o_begin<0){o_begin=o_begin+2*PI; };
       if(o_end>=2*PI){o_end=o_end-2*PI;}
       else if(o_end<0){o_end=o_end+2*PI; };
     
       float deta=o_begin-o_end;
        //标准化角度
       if(deta>=2*PI){deta=deta-2*PI;}
       else if(deta<0){deta=deta+2*PI; };

       float distance=A.R * deta;

       cout<<"圆心指向开始点的角度为"<<o_begin*180/PI<<endl;
       cout<<"圆心指向结束点的角度为"<<o_end*180/PI<<endl;
       cout<<"角度差为(逆时针为360-角)"<<deta*180/PI<<","<<deta<<endl;
       cout<<"半径为"<<A.R<<endl;
      
       if(A.type=="+"){
          cout<<"圆的类型为顺时针圆，走过的路径长度为:"<<distance<<endl<<endl;
       };
       if(A.type=="-"){
          distance=2*PI*A.R-distance;
          cout<<"圆的类型为逆时针圆，走过的路径长度为:"<<distance<<endl<<endl;
       };
       return distance;
      };
      
     all_points get_all_point(float deta,circle A,UAV C){   //,deta为变化的单位路程，A输入圆的信息,和C无人机的信息，输出时刻对应的坐标

        //初始化参数
        all_points B;
        C.fai=C.fai*PI/180; //将角度转化为弧度
        float z_deta=tan(C.fai)*deta;  //由x的距离变化计算z的单位距离变化
        float theat_deta=deta/A.R;            //由x的距离变化计算圆上圆心角的单位距离变化（弧度）
        float directy_theat=atan2(C.y[1]-A.end_point[1],C.x[1]-A.end_point[0]);  //圆上终点（切点）到目标点角度（弧度）
        float panxuan_point_x=C.x[1]+C.R_xy*cos(directy_theat-PI/2); //盘旋圆心横坐标
        float panxuan_point_y=C.y[1]+C.R_xy*sin(directy_theat-PI/2); //盘旋圆心纵坐标
        float panxuan_theat = directy_theat+PI/2; //盘旋点指向终点开始的角度
        float directly_deta_x=deta*cos(directy_theat);     //定义直线段x单位变化和y单位变化
        float directly_deta_y=deta*sin(directy_theat);
        float circle_distance=get_circle_distance(A); //得到圆的路径长度
        float o_begin=atan2(A.bigin_point[1]-A.circle_point[1],A.bigin_point[0]-A.circle_point[0]) ;//圆心指向开始点的向量角度
        if(o_begin>=2*PI){o_begin=o_begin-2*PI;}
        else if(o_begin<0){o_begin=o_begin+2*PI; }; //标准化角度
        int more_circle=0;
        int k=1; //直线段计数器
        int l=1; //盘旋段计数器
        int j=0; //z是否满足要求判断
        float all_distance=circle_distance+sqrt((A.end_point[0]-C.x[1])*(A.end_point[0]-C.x[1])+(A.end_point[1]-C.y[1])*(A.end_point[1]-C.y[1]));//计算xy平面总距离
        float all_z_deta=tan(C.fai)*all_distance; //按照二维走到终点，纵轴总变化距离
        int type; //根据到目标点是否降到平面判断类型
  
        //选择输出参数。
        cout<<"圆的长+直线长等于总长："<<circle_distance<<"+"<<sqrt((A.end_point[0]-C.x[1])*(A.end_point[0]-C.x[1])+(A.end_point[1]-C.y[1])*(A.end_point[1]-C.y[1]))<<"="<<all_distance<<endl;
        cout<<"盘旋点圆心坐标为：["<<panxuan_point_x<<","<<panxuan_point_y<<"]"<<endl;
        cout<<"走完xy下降的高度为"<<all_z_deta<<endl;
        cout<<"俯仰角（弧度）："<<C.fai<<endl;
        cout<<"z的单位变化为"<<z_deta<<endl;

        if(abs(C.z[1]-C.z[0])<=all_z_deta){  //如果提前到达终点，定义为1类型
            type=1;
            cout<<"类型为提前到达高度"<<endl;
        }
        else{    //如果需要盘旋绕圈，定义为2类型
            type=2; 
           float circle_z_deta = (2*PI*C.R_xy)*tan(C.fai);
           int more_circle=(int)((abs(C.z[1]-C.z[0])-all_z_deta)/circle_z_deta)+1; //定义盘旋的圈速 
           cout<<"类型为需要盘旋，盘旋的圈数为："<<more_circle<<endl;
           cout<<"盘旋走一圈的长度："<<2*PI*C.R_xy<<endl;
           cout<<"盘旋走一圈下降的高度："<<circle_z_deta<<endl;
        }

        if(type==1){  //对于提前结束的类型来说
          if(A.type=="+"){  //对于顺时针圆来说
           for(int i=0;;i++){
               if(i*deta<circle_distance){ //在圆上变化时
                 B.x.push_back(A.circle_point[0]+A.R*cos(o_begin-i*theat_deta));
                 B.y.push_back(A.circle_point[1]+A.R*sin(o_begin-i*theat_deta));
                };
                if(i*deta>=circle_distance){ //在直线段时候
                  B.x.push_back(A.end_point[0]+directly_deta_x*k);
                  B.y.push_back(A.end_point[1]+directly_deta_y*k);   
                  k=k+1;       
                };
                if(C.z[0]>=C.z[1]){ //出发点高度大于目标点
                    //到达目标高度后保持
                     if(i>=1 && B.z[i-1]-z_deta<=C.z[1]){  //到达目标高度后保持
                       B.z.push_back(C.z[1]);
                       j=1;
                       }
                       else{
                       B.z.push_back(C.z[0]-z_deta*i);
                       }
                 
                };
                 if(C.z[0]<C.z[1]){ //出发点高度小于目标点
                
                   
                      if(i>=1 && B.z[i-1]+z_deta<=C.z[1]){  //到达目标高度后保持
                       B.z.push_back(C.z[1]);
                       j=1;
                       }
                       else{
                       B.z.push_back(C.z[0]+z_deta*i);
                       }
                      
                  };
                if(i*deta>=all_distance){
                 break;
                };
            };
          }; //顺时针圆结束
          if(A.type=="-"){  //对于逆时针圆来说
           for(int i=0;;i++){
               if(i*deta<circle_distance){ //在圆上变化时
                 B.x.push_back(A.circle_point[0]+A.R*cos(o_begin+i*theat_deta));
                 B.y.push_back(A.circle_point[1]+A.R*sin(o_begin+i*theat_deta));
                };
                if(i*deta>=circle_distance){ //在直线段时候
                 B.x.push_back(A.end_point[0]+directly_deta_x*k);
                 B.y.push_back(A.end_point[1]+directly_deta_y*k);        
                  k=k+1;       
                };
                if(C.z[0]>=C.z[1]){ //出发点高度大于目标点
                   
                   if(i>=1 && B.z[i-1]-z_deta<=C.z[1]){  //到达目标高度后保持
                      B.z.push_back(C.z[1]);
                      j=1;
                      }
                      else{
                      B.z.push_back(C.z[0]-z_deta*i);
                      };
                };
                 if(B.z[0]<B.z[1]){ //出发点高度小于目标点
                   
                     if(i>=1 && B.z[i-1]+z_deta<=C.z[1]){  //到达目标高度后保持
                       B.z.push_back(C.z[1]);
                       j=1;
                       }
                       else{
                       B.z.push_back(C.z[0]+z_deta*i);
                       }
                };
                if(i*deta>=all_distance){
                 break;
                };
            };
           };//逆时针圆到此
         }; //提前结束类型圆到此为止

        
         
        if(type==2){  //对于未提前结束的圆来说 
            if(A.type=="+"){  //对于顺时针圆来说
               for(int i=0;;i++){
                  if(i*deta<circle_distance){ //在圆上变化时
                     B.x.push_back(A.circle_point[0]+A.R*cos(o_begin-i*theat_deta));
                     B.y.push_back(A.circle_point[1]+A.R*sin(o_begin-i*theat_deta));
                     };
                  if(i*deta>=circle_distance && i*deta < all_distance){ //在直线段时候
                     B.x.push_back(A.end_point[0]+directly_deta_x*k);
                     B.y.push_back(A.end_point[1]+directly_deta_y*k);
                     k=k+1;       
                     };
                  if(C.z[0]>=C.z[1]){ //出发点高度大于目标点
                      
                     if(i>=1 && B.z[i-1]-z_deta<=C.z[1]){  //到达目标高度后保持
                        B.z.push_back(C.z[1]);
                        j=1; //改变判断器
                     }
                     else{
                      B.z.push_back(C.z[0]-z_deta*i); 
                     };
                  };
                  if(C.z[0]<C.z[1]){ //出发点高度小于目标点
                     
                     if(i>=1 && B.z[i-1]+z_deta>=C.z[1]){  //到达目标高度后保持
                        B.z.push_back(C.z[1]);
                        j=1;
                     }
                     else{
                         B.z.push_back(C.z[0]+z_deta*i); 
                     };
                  };
                  if(i*deta>=all_distance){ //经过目标点但是高度未达到要求

                        B.x.push_back(panxuan_point_x+A.R*cos(panxuan_theat-l*theat_deta));
                        B.y.push_back(panxuan_point_y+A.R*sin(panxuan_theat-l*theat_deta)); 
                        l=l+1;
                  };
                  float final_deta = sqrt((B.x[i-1]-C.x[1])*(B.x[i-1]-C.x[1])+(B.y[i-1]-C.y[1])*(B.y[i-1]-C.y[1])  );
                  if(j==1 && final_deta <=deta){  //总距离大于多的圈数
                     break;
                  }
               };
            };
          if(A.type=="-"){  //对于逆时针圆来说
           for(int i=0;;i++){
               if(i*deta<circle_distance){ //在圆上变化时
                 B.x.push_back(A.circle_point[0]+A.R*cos(o_begin+i*theat_deta));
                 B.y.push_back(A.circle_point[1]+A.R*sin(o_begin+i*theat_deta));
                };
                if(i*deta>=circle_distance && i*deta < all_distance){ //在直线段时候
                 B.x.push_back(A.end_point[0]+directly_deta_x*k);
                 B.y.push_back(A.end_point[1]+directly_deta_y*k);     
                 k=k+1;       
                };
                if(C.z[0]>=C.z[1]){ //出发点高度大于目标点
                  
                  if(i>=1 && B.z[i-1]-z_deta<=C.z[1]){  //到达目标高度后保持
                     B.z.push_back(C.z[1]);
                     j=1; //改变判断器
                     }
                     else{
                        B.z.push_back(C.z[0]-z_deta*i); 
                     };
                  };
                if(C.z[0]<C.z[1]){ //出发点高度小于目标点
                  
                  if(i>=1 && B.z[i-1]+z_deta>=C.z[1]){  //到达目标高度后保持
                     B.z.push_back(C.z[1]);
                     j=1;
                     }
                     else{
                        B.z.push_back(C.z[0]+z_deta*i); 
                     };
                  };
                if(i*deta>=all_distance){ //经过目标点但是高度未达到要求
                  B.x.push_back(panxuan_point_x+A.R*cos(panxuan_theat-l*theat_deta));
                  B.y.push_back(panxuan_point_y+A.R*sin(panxuan_theat-l*theat_deta)); 
                  l=l+1;
                  };
                float final_deta = sqrt((B.x[i-1]-C.x[1])*(B.x[i-1]-C.x[1])+(B.y[i-1]-C.y[1])*(B.y[i-1]-C.y[1])  );
                if(j==1 && final_deta <=deta){  //总距离大于多的圈数
                     break;
                  }
            };
         };
        }
        
        
        
          return B;
     }
     
     void creat_log(all_points B){
          int length=B.x.size();
          
          // 打开一个输出文件流，用于写入CSV文件
           ofstream outFile("D:\\dubins_3d\\dubins_3d\\log_1UAV\\log_1UAV.csv");
 
          // 检查文件是否成功打开
           if (!outFile.is_open()) {
            cerr << "Error opening file!" << endl;
           }
           outFile<<"x"<<","<<"y"<<","<<"z"<<endl; //写入标题
              // 将三个数组的数据写入CSV文件
           for (size_t i = 0; i < length; ++i) {
              outFile << B.x[i]<<","<<B.y[i]<<","<<B.z[i]<<endl;
            }
            outFile<<endl;    
            outFile.close();
 
       cout << "Data has been written to log.csv" << endl;
     };

     public:
        
      all_points test(UAV A,float deta){  //可以得到坐标
        UAV B=coordinate(A);    //坐标转化得到A
        cout<<"转化后的rxy"<<B.R_xy<<endl;
        circle shun_circle = path_circle_shun(B);   //得到顺时针圆
        circle ni_circle =path_circle_ni(B);     //得到逆时针圆
        float shun_distance = get_circle_distance(shun_circle);  //顺指针圆路长
        float ni_distance = get_circle_distance(ni_circle);   //逆时针圆路长
        circle shortest_circle;
        if(shun_distance>=ni_distance){shortest_circle =ni_circle;cout<<"逆时针圆更短"<<endl<<endl;}
        else{shortest_circle =shun_circle;cout<<"顺时针圆更短"<<endl<<endl;};
        all_points C=get_all_point(deta,shortest_circle,B);
        //creat_log(C,string csv_path);
        creat_log(C);
        return C;
       };
       
      //  int meet_time(UAV A, UAV B, float check_disatance){ //输入无人机a和无人机b，以及检测间距,并且保存日志
      //    all_points a= test(A);
      //    creat_log(a,data_A);
      //    all points b = test(B);
      //    creat_log(a,data_B);
      //    int length=min(a.x.size(),b.x.size());
      //    int meet_num=-1;
      //    for(int i=0;i<length;i++)
      //     {
      //         float distance = sqrt((a.x[i]-b.x[i])*(a.x[i]-b.x[i])+(a.y[i]-b.y[i])*(a.y[i]-b.y[i])+(a.z[i]-b.z[i])*(a.z[i]-b.z[i]));
      //         if(distance <=check_distance){
      //           meet_num=i;
      //           break;
      //         };

      //     }
      //      return meet_num;
      //  }

};

int main()
{

  UAV test_UAV;
 

  //初始化测试数组的数据1
  test_UAV.x[0]=-1;
  test_UAV.y[0]=-1;
  test_UAV.x[1]=10;
  test_UAV.y[1]=0;
  test_UAV.theat=160; //输入为角度值

  test_UAV.fai=20; //输入为角度值
  test_UAV.R_xy=2;
  test_UAV.z[0]=100;
  test_UAV.z[1]=80;  
  
  dubins_cheak A;
  A.test(test_UAV,0.01);

 
  
  return 0;
}