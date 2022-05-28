from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as axes3d
import os


# Robot class that saves DH parameters
class Robot:
    def __init__(self, name, theta, a, d, alpha):
        self.name = name
        self.theta = theta
        self.a = a
        self.d = d
        self.alpha = alpha


# Save the DH parameters from a text file
def read_robot_params(param_filename):
    # Find text file with robot parameters
    cwd = os.getcwd()
    filename = cwd + '/robot_sim/' + param_filename

    theta = []
    a = []
    d = []
    alpha = []

    lines = 0
    with open(filename) as file:
        while (line := file.readline().rstrip()):
            if lines == 0:
                name = line
            if lines > 1:
                params = line.split()
                theta.append(float(params[0]))
                a.append(float(params[1]))
                d.append(float(params[2]))
                alpha.append(float(params[3]))
            lines+=1

    robot = Robot(name, theta, a, d, alpha)

    return robot


# forward kinematics
def FK(rbt):

    theta = robot.theta
    a = robot.a
    d = robot.d
    alpha = robot.alpha

    T = np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])


# Main function
param_filename = 'ur10e_parameters.txt'
robot = read_robot_params(param_filename)

print(robot.theta)

# Attaching 3D axis to the figure
fig = plt.figure()
elev = 35
azim = 60
ax = fig.add_subplot(111, projection="3d", elev=elev, azim=azim)
ax.set_position([0, 0, 0.95, 1])

# Setting the axes properties
ax.set_xlim3d([-1.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-1.0, 1.0])
ax.set_zlabel('Z')

ax.set_title('3D Test')

'''
# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 25, fargs=(data, lines),
                                   interval=50, blit=False)
'''

# Make the grid
x, y, z = np.meshgrid(np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.8))

# Make the direction data for the arrows
u = np.sin(np.pi * x) * np.cos(np.pi * y) * np.cos(np.pi * z)
v = -np.cos(np.pi * x) * np.sin(np.pi * y) * np.cos(np.pi * z)
w = (np.sqrt(2.0 / 3.0) * np.cos(np.pi * x) * np.cos(np.pi * y) *
     np.sin(np.pi * z))

ax.quiver(x, y, z, u, v, w, length=0.1, normalize=True)

'''
V = np.array([[1,1], [-2,2], [4,-7]])
origin = np.array([[0, 0, 0],[0, 0, 0]]) # origin point

plt.quiver(*origin, V[:,0], V[:,1], color=['r','b','g'], scale=21)
'''
# ani.save('double_pendulum.mp4', fps=15)
plt.show()