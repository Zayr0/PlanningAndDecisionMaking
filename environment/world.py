import pybullet as p
import pybullet_data
import time
import cvxpy as cp
import numpy as np
from stl import mesh
import os


class Environment:
    def __init__(self):
        self.width = 10
        self.height = 10
        self.world_size = np.array([[-2, 2], [-2, 2]])
        self.obstacles = []
        self.build_world()
        self.generate_random_obstacles(1, np.array([1, 1]))

    def build_world(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF("plane.urdf")

        #p.setAdditionalSearchPath(os.path.join(os.getcwd(), "environment", "assets"))
        #leonshouse = p.loadURDF("leonshouse.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
        #p.changeDynamics(leonshouse, -1, collisionFilterGroup=8, collisionFilterMask=9)
        #p.changeVisualShape(leonshouse, -1, rgbaColor=[0, 1, 0, 0.1])
        #p.setAdditionalSearchPath(pybullet_data.getDataPath())
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
        temp_stl_filename = "temp/temp_mesh.stl"
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
        # randomized polygon approach
        # n_gon = 3  # np.random.choice([3,4,5], 1)  # n of vertices of prism base
        # height = 3
        # radius = 0.3
        # vertices_bottom = [[radius, 0, 0]]
        # vertices_top = [[radius, 0, height]]
        # d_angle = (n_gon - 2) * np.pi / n_gon
        # for i in range(n_gon - 1):
        #     rot_mat = np.array([[np.cos(d_angle * (i + 1)), - np.sin(d_angle * (i + 1)), 0],
        #                         [np.sin(d_angle * (i + 1)), np.cos(d_angle * (i + 1)), 0],
        #                         [0, 0, 1]])
        #     vertices_bottom.append(list(rot_mat @ np.array(vertices_bottom[0])))
        # for i in range(n_gon):
        #     vertices_top.append(list(vertices_bottom[i] + np.array([0, 0, height])))
        #
        # data = np.zeros(n_gon + 2, dtype=mesh.Mesh.dtype)
        #
        # for i in range(n_gon - 1):
        #     data['vectors'][i] = np.vstack([vertices_bottom[i], vertices_bottom[i + 1], vertices_top[i]])
        # data['vectors'][n_gon] = np.vstack(vertices_top[:3])
        # data['vectors'][n_gon + 1] = np.vstack(vertices_bottom[:3])
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


class Obstacle:
    def __init__(self, A, b):
        self.center = 0
        self.id = 0
        self.vertices = []
        self.constr_mat_A = A
        self.constr_mat_b = b

    def _min_dist(self, constr_j_A, constr_j_b):
        """this formulates the QP optimization to obtain the shortest distance between
        the polytope i of this obstacle and another polytope j"""
        zi = cp.Variable(2)
        zj = cp.Variable(2)

        cost = cp.quad_form(zi - zj, np.eye(2)) #cp.quad_form(zi, np.eye(2)) - cp.quad_form(zj, np.eye(2))
        constraints = []
        constraints += [constr_j_A @ zj <= constr_j_b]
        constraints += [self.constr_mat_A @ zi <= self.constr_mat_b]

        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP)

        dist = abs(zi.value - zj.value)
        return np.linalg.norm(dist, 2)

# testing:
Ai = np.array([[1, 0],
               [-1, 0],
               [0, 1],
               [0, -1]])
bi = np.array([1, 0, 1, 0])

Aj = np.array([[1, 0],
               [-1, 0],
               [0, 1],
               [0, -1]])
bj = np.array([3, -2, 1, 0])

obs1 = Obstacle(Ai, bi)
print(obs1._min_dist(Aj, bj))