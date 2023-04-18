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
    plt.xlabel ="x"
    plt.ylabel ="y"
    plt.zlabel ="z"
    plt.show()

    z_ax = np.array([0, 0, 1])

    rot_axis = np.cross(z_ax, n)

    rot_angle = -acos( np.dot(n,z_ax) / ( np.linalg.norm(n) * np.linalg.norm(z_ax) ) )

    print(rot_angle*180/(2*pi))

    # normalize vector
    rot_axis = rot_axis/np.linalg.norm(rot_axis)

    q = [ cos(rot_angle/2), sin(rot_angle/2)*rot_axis ]

    print(q)

    

# Get the 3 points from the user
print("Point 1:")
#p0 = point_input()
p0 = np.array([0.588193 , -0.0629043,  0.00409937])

print("Point 2:")
#p1 = point_input()
p1 = np.array([0.301129, -0.0612265, -0.00146244])

print("Point 3:")
#p2 = point_input()
p2 = np.array([0.207729, 0.550682,0.00317771])

find_plain_equation(p0,p1,p2)
