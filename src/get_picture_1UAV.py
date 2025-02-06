import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib as mpl
from matplotlib import cm
from matplotlib import pyplot as plt

path1='D:\dubins_3d\dubins_3d\log_1UAV\log_1UAV.csv'
data1=pd.read_csv(path1)
x1=data1[['x']]
y1=data1[['y']]
z1=data1[['z']]


# 使用matplotlib绘制散点图
#绘制二维表格

plt.plot(x1,y1,color='b',linestyle='--',linewidth=1,marker='o',markeredgecolor='steelblue',markersize='1',label="UAV_A's PATH")
#plt.plot(x1[145],y1[145],marker='o',markeredgecolor='b',markersize='5')
# 添加标题和标签
plt.title('UAV_meet_check_2D')
plt.grid(True)
plt.axis("equal")
plt.legend(shadow=True)
# 显示图表
plt.show()


fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
# 绘制散点图
scatter1 = ax.scatter(x1, y1, z1,c='steelblue',marker='o',s=1,label="UAV_A 's Path")
ax.set_xlabel('X ')
ax.set_ylabel('Y ')
ax.set_zlabel('Z ')
plt.grid(True)  
plt.axis("equal")
plt.legend(shadow=True)
# 显示图表
plt.show()



