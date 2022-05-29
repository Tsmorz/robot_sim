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
def readRobotParams(param_filename):
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


# forward kinematics from DH formulat
def forwardKinematics(robot):

    theta = robot.theta
    a = robot.a
    d = robot.d
    alpha = robot.alpha

    T = []
    i = 0
    # loop through all joints
    for i in range(len(robot.theta)):
        T.append(DH(theta[i], a[i], d[i], alpha[i]))

    return T


# Denavit-Hartenberg Parameters
def DH(theta, a, d, alpha):
    T = [
        [cos(theta), -sin(theta)*cos(alpha),
            sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha),
            -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
        ]
    return T


# modified Denavit-Hartenberg Parameters
def modDH(theta, a, d, alpha):
    T = [
        [cos(theta), -sin(theta),
            0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha),
            -sin(alpha), -d*sin(alpha)],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha),
            cos(alpha), d*cos(alpha)],
        [0, 0, 0, 1]
        ]
    return T


# Jacobian matrix from DH parameters
def jacobian(robot):

    return False


# Inverse Kinematics
def inverseKinematics(robot):

    return False


# draw x-y-z axes at joints
def drawJointAxes(fwd_kin):

    # draw unit axes
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', length=0.1, normalize=True)
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=0.1, normalize=True)
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=0.1, normalize=True)

    unit_x = [1, 0, 0, 0]
    unit_y = [0, 1, 0, 0]
    unit_z = [0, 0, 1, 0]
    origin = [0, 0, 0, 1]

    for i in range(len(fwd_kin)):
        if i==0:
            fwd_kin_chain = fwd_kin[0]
        else:
            fwd_kin_chain = np.matmul(fwd_kin_chain, fwd_kin[i])

        x, y, z, one = np.matmul(fwd_kin_chain, np.transpose(origin))

        xu, xv, xw, one = np.matmul(fwd_kin_chain, np.transpose(unit_x))
        yu, yv, yw, one = np.matmul(fwd_kin_chain, np.transpose(unit_y))
        zu, zv, zw, one = np.matmul(fwd_kin_chain, np.transpose(unit_z))

        if i==5:
            ax.quiver(x, y, z, xu, xv, xw, color='r', length=0.1, normalize=True)
            ax.quiver(x, y, z, yu, yv, yw, color='g', length=0.1, normalize=True)
            ax.quiver(x, y, z, zu, zv, zw, color='b', length=0.1, normalize=True)


# Inverse of homogeneous transformation
def invHomoTransform(matrix):
    rot = matrix[0:3, 0:3]
    tran = matrix[0:3, 3]

    inverse = np.diag(4)

    return inverse


# animate the change in joint angles
def animate(robot):
    
    robot.theta[0] = [robot.theta[0] + 0.1]
    fwd_kin = forwardKinematics(robot)

    drawJointAxes(fwd_kin)
    return robot


# ----- Main function --------------------------------
param_filename = '3R.txt'
param_filename = 'ur16e.txt'
#param_filename = 'ur5e.txt'
#param_filename = '5R1P.txt'


# Attaching 3D axis to the figure
fig = plt.figure()
elev = 35
azim = 60
ax = plt.axes(projection="3d", elev=elev, azim=azim)

# Setting the axes properties
ax.set_xlim3d([-1.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-1.0, 1.0])
ax.set_zlabel('Z')


robot = readRobotParams(param_filename)
ax.set_title(robot.name)
fwd_kin = forwardKinematics(robot)
drawJointAxes(fwd_kin)


for i in range(50):
    for j in range(2):
        robot.theta[j] = robot.theta[j] + 2*np.pi/50
    fwd_kin = forwardKinematics(robot)
    
    '''
    plt.cla()
    # Setting the axes properties
    ax.set_xlim3d([-1.0, 1.0])
    ax.set_xlabel('X')

    ax.set_ylim3d([-1.0, 1.0])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-1.0, 1.0])
    ax.set_zlabel('Z')
    '''
    drawJointAxes(fwd_kin)
    plt.draw()
    plt.pause(0.01)

'''
ani = animation.FuncAnimation(fig, animate(robot), np.arange(1, 400),
                              interval=25, blit=True)
'''

# ani.save('double_pendulum.mp4', fps=15)

plt.show()