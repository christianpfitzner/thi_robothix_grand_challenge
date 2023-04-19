import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import sin, cos, acos, asin, pi

# Input from 
def point_input():

    x = float(input("  x: "))
    y = float(input("  y: "))
    z = float(input("  z: "))

    return np.array([x, y, z])

def find_plain_equation(A, B, C):

    # Calculate the vector 0 to A
    OA = A

    # Calculate the vector from A to B
    AB = B - A

    # Calculate the vector from A to C
    AC = C - A

    # Do a cross produkt for the coefficients
    # in front of x,y,z
    # aka Normal Vector (Z-Axis)
    n = np.cross(AB,AC)*(-1)

    # Vector dot produkt for the offset k
    k = np.dot(n,OA)

    print("Equation: ", n[0],"x +", n[1], "y +", n[2],"z + ",k,sep="")

    x = np.linspace(-1,1,10)
    y = np.linspace(-1,1,10)

    X,Y = np.meshgrid(x,y)

    # Calcualting Z from the equation
    Z = (n[0]*X + n[1]*Y + k)/n[2]

    fig = plt.figure()
    plt3d = fig.gca(projection='3d')
    plt3d.plot_surface(X, Y, Z)
    plt.show()

    z_ax = np.array([0, 0, 1])

    rot_axis = np.cross(z_ax, n)

    rot_angle = acos( np.dot(z_ax,n) / ( np.linalg.norm(n) * np.linalg.norm(z_ax) ) )

    print(rot_angle*180/(2*pi))

    

# Get the 3 points from the user
print("Point 1:")
#p0 = point_input()
p0 = np.array([2 ,-2, 4.5])

print("Point 2:")
#p1 = point_input()
p1 = np.array([-2, 3, 0])

print("Point 3:")
#p2 = point_input()
p2 = np.array([0, 3, -1.5])

find_plain_equation(p0,p1,p2)
