import pybullet as p
import pybullet_data
import time
from environment.world import Environment
import numpy as np

from Sampler import Sampler
from Helper.Bounds import Bounds

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, 0)

env = Environment()

#Define bounds for sampling
size = np.array([[-5, 5], [-5, 5], [0, 1.5]])
bounds = Bounds(size, center=[0, 0, 0])
sampler = Sampler()

bounds.drawBounds()

while (p.isConnected()):
    time.sleep(1. / 240.)
    hits, misses = sampler.sampleSpace(10, bounds, env.obstacles)






