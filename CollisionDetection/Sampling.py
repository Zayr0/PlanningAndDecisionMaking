import pybullet as p
import pybullet_data
import time
from environment.world import Environment

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

env = Environment()

p.setGravity(0, 0, 0)
N = 1000

while (p.isConnected()):
    time.sleep(1. / 240.)
    for obId in env.obstacles:
        pos, orn = p.getBasePositionAndOrientation(obId)
        print(pos)