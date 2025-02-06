
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