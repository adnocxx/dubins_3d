# dubins_3d
 Flight control of two aircraft Dobbins path planning collision detection

本程序包含以下内容：    
单机版（1UAV）：对无人机的路径进行三维规划  
双机版（2UAV）：对两个无人机规划的路径进行碰撞检测

## src
### dubins_3d_1UAV

在main函数中修改基本信息：

按照UAV结构体中的说明进行更改       

    get_picture_1UAV  struct UAV{ 
        float x[2],y[2],z[2],R_xy,v,theat,fai;
        //x,y,z为无人机的三维坐标,包括开始和结尾
        //R_xy为平面最小转弯半径，由偏航角决定
        //v为无人机速度，假设为匀速运动
        //theat为初始偏航角，假设未有俯仰角和滚转角
        //fai是最大俯仰角
    }；
运行结束后路径点坐标保存在log_1VAU中
### get_picture_1UAV
直接运行即可,可以绘制单集版的路径图，先显示2d，关闭后显示3d

（双机版同理）