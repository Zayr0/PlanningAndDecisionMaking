import pybullet as p
import pybullet_data
import time
from environment.world import Environment
import numpy as np
from scipy.optimize import linprog

def in_hull(points, x):
    n_points = len(points)
    n_dim = len(x)
    c = np.zeros(n_points)
    A = np.r_[points.T,np.ones((1,n_points))]
    b = np.r_[x, np.ones(1)]
    lp = linprog(c, A_eq=A, b_eq=b)
    return lp.success

def findAllVerts(obId):
    v, i = p.getMeshData(obId, -1)
    pos, orn = p.getBasePositionAndOrientation(obId)
    verts = np.asarray(i)
    all_verts = verts + np.tile(np.asarray(pos), (verts.shape[0], 1))
    return all_verts


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

env = Environment()

collisionFilterGroup = 0
collisionFilterMask = 0
enableCollision = 1
noCollisions = 0

samplerId = p.loadURDF("sphere2.urdf", [0, 0, 3],  useMaximalCoordinates=False)
p.setCollisionFilterGroupMask(samplerId, -1, collisionFilterGroup, collisionFilterMask)

groundId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
p.setCollisionFilterGroupMask(groundId, -1, collisionFilterGroup, collisionFilterMask)

p.setCollisionFilterPair(groundId, samplerId, -1, -1, noCollisions)

for obId in env.obstacles:
    p.setCollisionFilterGroupMask(obId, -1, collisionFilterGroup, collisionFilterMask)
    p.setCollisionFilterPair(samplerId, obId, -1, -1, enableCollision)
    p.setCollisionFilterPair(groundId, obId, -1, -1, enableCollision)


p.setRealTimeSimulation(1)
p.setGravity(0, 0, 0)

offset = 0.5

while (p.isConnected()):
    time.sleep(1. / 240.)

    x = (env.world_size[0][1] - env.world_size[0][0]) * (np.random.rand() - 0.5)
    y = (env.world_size[1][1] - env.world_size[1][0]) * (np.random.rand() - 0.5)

    for obId in env.obstacles:
        pos, orn = p.getBasePositionAndOrientation(obId)
        # p.resetBasePositionAndOrientation(samplerId, [pos[0] + offset, pos[1] + offset, pos[2] + offset], orn)
        # contactPoints = p.getContactPoints(samplerId, obId, -1, -1)
        # print(contactPoints)

        # min, max = p.getAABB(obId, -1)
        allVerts = findAllVerts(obId)
        # point = [pos[0] + offset, pos[1] + offset, pos[2] + offset]

        point = [x, y, 0.5]
        val = in_hull(allVerts, point)

        if val:
            print('point inside object:', obId)

        p.addUserDebugPoints([point], [[0, 0, 1]], 5)





