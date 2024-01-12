import pybullet as p
import pybullet_data
import time
import numpy as np
from stl import mesh
from scipy.spatial import ConvexHull
import os
from Environment.Obstacle import Obstacle
from Environment.MovingSpheres import MovingSpheres


class Environment:
    def __init__(self, numObstacles=0, type="Static", basePosition=[0,0,0]):
        self.width = 10
        self.depth = 10
        self.height = 10

        self.basePosition = basePosition
        self.numObstacles = numObstacles
        self.obstacles = []

        if type == "Static":
            self.build_static_world()
        elif type == "Dynamic":
            self.build_dynamic_world()

    def build_dynamic_world(self):
        radius = 1.0
        left2Spawn = self.numObstacles

        while left2Spawn > 0:
            position = np.asarray(self.basePosition) + np.random.uniform(
                low=[-int(self.width / 2), -int(self.depth / 2), 0],
                high=[int(self.width / 2), int(self.depth / 2), self.height], size=(1, 3))

            positionFree = True

            for ob in self.obstacles:
                surfaceDist = ob.center_dist(position) - 2*radius
                if surfaceDist <= 0.0:
                    positionFree = False

            if positionFree:
                left2Spawn -= 1
                sphereID = p.loadURDF("Environment/VisualSphere.urdf", position[0], useMaximalCoordinates=False)
                p.changeVisualShape(sphereID, -1, rgbaColor=[0, 0, 1, 0.9])
                self.obstacles.append(MovingSpheres(ID=sphereID, radius=radius, position=position[0]))

    def build_static_world(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # build some obstacles
        for x in np.arange(- int(self.width/2), int(self.width/2), 2):
            for y in np.arange(- int(self.depth/2), int(self.depth/2), 2):
                for z in np.arange(0, int(self.height), 2):
                    initial_pose = [x + self.basePosition[0], y + self.basePosition[1], z + self.basePosition[2]]
                    self.generate_obstacle(initial_pose)
        return

    def generate_obstacle(self, initial_pose, hash=0):
        x_off, y_off, z_off = initial_pose
        # Define the intervals for random points
        xmin, xmax = -1.0, 1.0
        ymin, ymax = -1.0, 1.0
        zmin, zmax = -1.0, 2

        # Generate random 3D points within specified intervals
        #np.random.seed(42)
        #points_base = np.array([[0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, 0.5, 0], [-0.5, -0.5, 0]])
        points_random = np.random.uniform(low=[xmin, ymin, zmin], high=[xmax, ymax, zmax], size=(7, 3))
        #points = np.vstack([points_base, points_random])
        points = points_random
        points = points + np.tile(np.array([x_off, y_off, z_off]), (points.shape[0], 1))
        # Compute the convex hull
        hull = ConvexHull(points)

        # Get the convex hull vertices
        hull_vertices = points[hull.vertices]
        # Create an STL mesh from the convex hull vertices
        stl_mesh = mesh.Mesh(np.zeros(hull_vertices.shape[0], dtype=mesh.Mesh.dtype))
        for i, vertex in enumerate(hull_vertices):
            stl_mesh.vectors[i] = vertex

        # Save the STL file
        temp_stl_filename = "temp/temp_mesh_convex_hull" + str(time.time()) + ".stl"
        stl_mesh.save(temp_stl_filename)

        # Load the mesh using PyBullet
        mesh_collision = p.createCollisionShape(p.GEOM_MESH, fileName=temp_stl_filename)
        os.remove(temp_stl_filename)
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # orientation as quaternion

        # Create a multi-body with the collision shape
        obstacle_body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=mesh_collision,
            basePosition=[0, 0, 0],
            baseOrientation=initial_orientation,
        )
        p.changeVisualShape(obstacle_body_id, -1, rgbaColor=[1, 0, 0, 1])  # red color
        obs1 = Obstacle(obstacle_body_id)
        obs1.constr_mat_A, obs1.constr_mat_b = hull.equations[:, :-1], -1 * hull.equations[:,-1]
        self.obstacles.append(obs1)
        return

    def distance_nearest_obstacle(self, current_pos):
        dist_min = np.inf
        for obs in self.obstacles:
            dist_new = obs.min_dist(current_pos)
            dist_min = np.min([dist_min, dist_new])
        return dist_min



