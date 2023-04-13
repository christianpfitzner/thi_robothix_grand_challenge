import numpy as np

# Input from 
def point_input():

    x = float(input("  x: "))
    y = float(input("  y: "))
    z = float(input("  z: "))

    return np.array([x, y, z])


def calc_origin(p1, p2, p3):
    # Calculate the vector from p1 to p2
    v1 = p2 - p1

    # Calculate the vector from p1 to p3
    v2 = p3 - p1

    # Calculate the cross product of v1 and v2
    v3 = np.cross(v1, v2)

    # unit vectors
    e_y = - v1 / np.linalg.norm(v1)
    e_z = v3 / np.linalg.norm(v3)
    e_x = np.cross(e_y, e_z)

    lambda1 = ((p1[1] - p3[1])*e_x[0] + (p3[0] - p1[0])*e_x[1])/(e_x[1]*e_y[0] - e_x[0]*e_y[1])

    origin = p1 + lambda1*e_y

    return origin


# Get the 3 points from the user
print("Point 1:")
p1 = point_input()

print("Point 2:")
p2 = point_input()

print("Point 3:")
p3 = point_input()

print("Origin: ", calc_origin(p1, p2, p3))