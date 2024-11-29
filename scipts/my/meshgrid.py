import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 设置球体的半径
radius = 1.0

# 生成球体的角度
phi = np.linspace(0, np.pi, 50)     # 纵向角度
theta = np.linspace(0, 2 * np.pi, 50)  # 横向角度
phi, theta = np.meshgrid(phi, theta)

# 将角度转换为球坐标 (x, y, z)
x = radius * np.sin(phi) * np.cos(theta)
y = radius * np.sin(phi) * np.sin(theta)
z = radius * np.cos(phi)

# 创建图形和3D轴
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# 绘制球体的表面
ax.plot_surface(x, y, z, color='b', edgecolor='k', alpha=0.7)

# 设置视角和标签
ax.set_title("3D Sphere")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# 显示图形
plt.show()
