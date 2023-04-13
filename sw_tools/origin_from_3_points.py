import numpy as np

# Get the 3 points from the user
print("Point 1:")
x = float(input("  x: "))
y = float(input("  y: "))
z = float(input("  z: "))
p1 = np.array([x, y, z])

print("Point 2:")
x = float(input("  x: "))
y = float(input("  y: "))
z = float(input("  z: "))
p2 = np.array([x, y, z])

print("Point 3:")
x = float(input("  x: "))
y = float(input("  y: "))
z = float(input("  z: "))
p3 = np.array([x, y, z])

# Calculate the vector from p1 to p2
v1 = p2 - p1

# Calculate the vector from p1 to p3
v2 = p3 - p1

# Calculate the cross product of v1 and v2
v3 = np.cross(v1, v2)

# unit vectors
e_x = v1 / np.linalg.norm(v1)
e_z = v3 / np.linalg.norm(v3)
e_y = np.cross(e_z, e_x)

lambda1 = ((p1[1] - p3[1])*e_x[0] + (p3[0] - p1[0])*e_x[1])/(e_x[1]*e_y[0] - e_x[0]*e_y[1])

origin = p1 + lambda1*e_x

print("Origin: ", origin)