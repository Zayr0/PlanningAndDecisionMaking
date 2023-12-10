import pybullet as p

p.connect(p.GUI)

# Create a simple box as an obstacle
obstacle_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, 1])
obstacle_body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacle_id, basePosition=[0, 0, 0])

# Sample point
sample_point = [0.5, 0.5, 1.]

# Set the start and end points of the ray
ray_start = [0, 0, 2]  # A point above the obstacle
ray_end = sample_point

# Perform ray test
result = p.rayTest(ray_start, ray_end)
p.addUserDebugPoints([ray_start], [[0, 0, 1]], 5)
p.addUserDebugPoints([ray_end], [[0, 0, 1]], 5)
# If the ray intersects with an object, the sample point is inside the obstacle
if result[0][0] == obstacle_body_id:
    print("Sample point is inside the obstacle")
    p.addUserDebugLine(ray_start, ray_end, [0, 1, 0], 3)
else:
    print("Sample point is not inside the obstacle")
    p.addUserDebugLine(ray_start, ray_end, [0, 1, 0], 1)
