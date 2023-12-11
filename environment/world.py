import pybullet as p
import pybullet_data
import time
import numpy as np
from stl import mesh


class Environment:
    def __init__(self):
        self.width = 10
        self.height = 10
        self.world_size = np.array([[-5, 5], [-5, 5]])
        self.obstacles = []
        self.build_world()
        self.generate_random_obstacles(10, np.array([1, 1]))

    def build_world(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF("plane.urdf")
        return

    def generate_random_obstacles(self, n_obstacles, bb_size):
        # np.random.seed(0)
        # Create 3 faces of a cube
        data = np.zeros(6, dtype=mesh.Mesh.dtype)

        # Top of the cube
        data['vectors'][0] = np.array([[0, 1, 1], [1, 0, 1], [0, 0, 1]])
        data['vectors'][1] = np.array([[1, 0, 1], [0, 1, 1], [1, 1, 1]])
        # Front face
        data['vectors'][2] = np.array([[1, 0, 0], [1, 0, 1], [1, 1, 0]])
        data['vectors'][3] = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 0]])
        # Left face
        data['vectors'][4] = np.array([[0, 0, 0], [1, 0, 0], [1, 0, 1]])
        data['vectors'][5] = np.array([[0, 0, 0], [0, 0, 1], [1, 0, 1]])

        # Create an STL mesh from the polygon vertices
        mesh_data = mesh.Mesh(data.copy())

        # Save the STL mesh to a temporary file
        temp_stl_filename = "temp_mesh.stl"
        mesh_data.save(temp_stl_filename)

        # Load the mesh using PyBullet
        mesh_collision = p.createCollisionShape(p.GEOM_MESH, fileName=temp_stl_filename)

        xs = np.random.choice(np.linspace(self.world_size[0][0], self.world_size[0][1], n_obstacles * 10), n_obstacles,
                              replace=False)
        ys = np.random.choice(np.linspace(self.world_size[1][0], self.world_size[1][1], n_obstacles * 10), n_obstacles,
                              replace=False)

        for i in range(n_obstacles):
            # Set the initial pose of the obstacle

            initial_pose = [xs[i], ys[i], 0]  # [x, y, z]
            initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # orientation as quaternion

            # Create a multi-body with the collision shape
            obstacle_body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=mesh_collision,
                basePosition=initial_pose,
                baseOrientation=initial_orientation,
            )
            # You can set additional properties for the obstacle if needed, such as color, friction, etc.
            p.changeVisualShape(obstacle_body_id, -1, rgbaColor=[1, 0, 0, 1])  # red color
            self.obstacles.append(obstacle_body_id)
        return

    def get_closest_point(self):
        # Load or create your collision object (e.g., a mesh)

        # Define a point in space
        target_point = [1, 2, 0]

        # Create a temporary collision object (a sphere) at the target point
        temp_sphere_id = p.createCollisionShape(p.GEOM_SPHERE, radius=0.01)
        temp_body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=temp_sphere_id, basePosition=target_point)

        for colID in self.obstacles:
            print(colID)
            # Get the closest points between the temporary object and the existing object
            result = p.getClosestPoints(temp_body_id, colID, distance=0.0)

            # Extract the position of the closest point on the existing object
            if result:
                closest_point_on_existing_object = result[0][6]

                print("Closest point on the existing object:", closest_point_on_existing_object)
            else:
                print("No closest point found")

        # Remove temporary objects
        p.removeBody(temp_body_id)