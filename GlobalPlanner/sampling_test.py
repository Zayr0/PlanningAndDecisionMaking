import pybullet as p
import random
import math
import numpy as np
from RRT import RRT

p.connect(p.GUI)

# Create a simple box as an obstacle
# obstacle_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, 1])
# obstacle_body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacle_id, basePosition=[0, 0, 0])

# # 创建透明球的图形形状，将 alpha 通道设置为 0.5（半透明）
# transparent_sphere_visual = p.createVisualShape(
#     shapeType=p.GEOM_SPHERE,
#     radius=1.0,
#     rgbaColor=[1, 0, 0, 0.1]  # RGB 颜色为红色，alpha 通道为0.5
# )
#
# # 创建多体系统
# transparent_sphere_body = p.createMultiBody(baseVisualShapeIndex=transparent_sphere_visual)

# Sample point
sample_point = [0.5, 0.5, 1.]

# Set the start and end points of the ray
start = np.array([0.4, -0.1, -0.9])  # A point above the obstacle
goal = np.array([0.8, 0.3, -0.5])
c_min = np.sqrt(np.sum((start-goal)**2))
c_max = 1.05*c_min
x_center = (start+goal)/2
C = RRT.rotation_matrix_from_direction(goal-start, c_min)


rrt = RRT(x_range=(-5, 5), y_range=(-5, 5), z_range=(0, 1), expandDis=1.0)
p.addUserDebugPoints([start], [[1, 0, 0]], 10)
p.addUserDebugPoints([goal], [[1, 0, 0]], 10)
p.addUserDebugLine(start, goal, [0, 1, 0], 2)
while True:
    rnd = rrt.ellipsoid_sample(c_max, c_min, x_center, C)
    p.addUserDebugPoints([rnd.tolist()], [[0, 0, 1]], 3)
