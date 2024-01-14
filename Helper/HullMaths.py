from scipy.spatial import ConvexHull
import numpy as np
import pybullet as p
from stl import mesh
import time
import os


def remove_duplicate_inequalities(A, b):
    """
    Removes duplicate inequalities from the convex hull equations.

    Parameters:
    A (numpy.ndarray): Coefficients of the inequalities.
    b (numpy.ndarray): Constants of the inequalities.

    Returns:
    numpy.ndarray: Unique coefficients of the inequalities.
    numpy.ndarray: Unique constants of the inequalities.
    """
    # Combine A and b to form a single array
    combined = np.hstack((A, b.reshape(-1, 1)))

    # Use np.unique to remove duplicates. Since np.unique works on 1D arrays,
    # we need to view the 2D array as a 1D array of structured elements.
    dtype = combined.dtype.descr * combined.shape[1]
    unique_combined = np.unique(combined.view(dtype), axis=0).view(combined.dtype).reshape(-1, combined.shape[1])

    # Split the array back into A and b
    unique_A = unique_combined[:, :-1]
    unique_b = unique_combined[:, -1]
    return unique_A, unique_b


def draw_polytope(points):
    print(points)
    hull = ConvexHull(points)

    # Vertices from the convex hull
    vertices = hull.points.tolist()

    # Face indices from the convex hull - ensuring proper flattening
    indices = []
    for simplex in hull.simplices:
        # Ensuring correct orientation (if needed)
        # simplex = correct_orientation(simplex, vertices)  # Implement this if needed
        indices.extend(simplex)

    # Create visual shape
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                          vertices=vertices,
                                          indices=indices,
                                          meshScale=[1, 1, 1],
                                          rgbaColor=[1, 0, 1, 0.1])  # Purple color, half-opacity

    # Create a multi-body with the visual shape
    body_id = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=-1,
                                baseVisualShapeIndex=visual_shape_id,
                                basePosition=[0, 0, 0])
    return body_id


def draw_polytope2(points):
    # Load the mesh using PyBullet
    mesh_collision = p.createCollisionShape(p.GEOM_MESH, vertices=points, meshScale=[1, 1, 1])
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # orientation as quaternion

    # Create a multi-body with the collision shape
    obstacle_body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=mesh_collision,
        basePosition=[0, 0, 0],
        baseOrientation=initial_orientation,
    )
    p.changeVisualShape(obstacle_body_id, -1, rgbaColor=[1, 0, 1, 0.1])  # red color
    return obstacle_body_id
