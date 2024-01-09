import numpy as np


class MovingSpheres:
    def __init__(self, radius=1.0, position=np.array([0.0, 0.0, 0.0]), velocity=np.array([None])):
        self.r = radius
        self.p = position
        self.v = velocity

    #Update and move the sphere.
    def update(self, dt):
        self.p += self.v*dt

    #This function returns the smallest distance from a position to the surface of the sphere.
    def min_dist(self, position):
        return np.linalg.norm(position-self.p)-self.r

    #This function returns the closest point on the surface to another position
    def closest_point_on_surface(self, position):
        c2c = (position - self.p)
        return self.r*c2c/np.linalg.norm(c2c) + self.p

    #This function calculates the vector from p1 to the closest point on the surface. (The minimal distance vector)
    def min_dist_vector(self, position):
        c2c = (position-self.p)
        return c2c - self.r*c2c/np.linalg.norm(c2c)

    #This function gives a hyperplane in the form of (a(x - x0) + b(y - y0) + c(z - z0) = 0)
    #In an array as [x0, y0, z0, a, b, c], based on normal vectorn and a surface point.
    def calculate_tangent_plane(self, position):
        point = self.closest_point_on_surface(position)
        n = position - point
        return np.array([point, n]) # Return plane
