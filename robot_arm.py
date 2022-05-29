from xml.dom.expatbuilder import theDOMImplementation
from numpy import linspace, sin, cos
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d
import os
import pprint
pp = pprint.PrettyPrinter(indent=4)

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

    theta = robot.theta
    a = robot.a
    d = robot.d
    alpha = robot.alpha

    homoTransform = []
    i = 0
    # loop through all joints
    for i in range(len(robot.theta)):
        T = DH(theta[i], a[i], d[i], alpha[i])
        homoTransform.append(T)

    return homoTransform


# draw x-y-z axes at joints
def getJointAxes(theta):
    x = []
    y = []
    z = []
    u = []
    v = []
    w = []
    color = []

    # origin axes
    for i in range(3):
        x.append(0)
        y.append(0)
        z.append(0)

        if i==0:
            u.append(1)
            v.append(0)
            w.append(0)
            color.extend('r')
        elif i==1:
            u.append(0)
            v.append(1)
            w.append(0)
            color.extend('g')
        else:
            u.append(0)
            v.append(0)
            w.append(1)
            color.extend('b')

    # robot axes
    unit_x = [1., 0., 0., 0.]
    unit_y = [0., 1., 0., 0.]
    unit_z = [0., 0., 1., 0.]
    origin = [0., 0., 0., 1.]

    robot.theta = theta
    fwd_kin = forwardKinematics(robot)

    for i in range(len(fwd_kin)):
        if i==0:
            fwd_kin_chain = fwd_kin[0]
        else:
            fwd_kin_chain = np.matmul(fwd_kin_chain, fwd_kin[i])

        tmp_x, tmp_y, tmp_z, one = np.matmul(fwd_kin_chain, np.transpose(origin))
        ux, vx, wx, one = np.matmul(fwd_kin_chain, np.transpose(unit_x))
        uy, vy, wy, one = np.matmul(fwd_kin_chain, np.transpose(unit_y))
        uz, vz, wz, one = np.matmul(fwd_kin_chain, np.transpose(unit_z))

        x.extend([tmp_x, tmp_x, tmp_x])
        y.extend([tmp_y, tmp_y, tmp_y])
        z.extend([tmp_z, tmp_z, tmp_z])
        u.extend([ux, uy, uz])
        v.extend([vx, vy, vz])
        w.extend([wx, wy, wz])
        color.extend(['r','g','b'])
    
    return x, y, z, u, v, w, color


def initPlot():
    ax.set_title(robot.name)

    # Setting the axes properties
    ax.set_xlim3d([-1.0, 1.0])
    ax.set_xlabel('X')
    ax.set_ylim3d([-1.0, 1.0])
    ax.set_ylabel('Y')
    ax.set_zlim3d([-1.0, 1.0])
    ax.set_zlabel('Z')


def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 9)
    theta = np.linspace(0, 2*np.pi, 9)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y

    return x_grid,y_grid,z_grid


# ----- Main function --------------------------------
param_filename = '3R.txt'
param_filename = 'ur16e.txt'
#param_filename = 'ur5e.txt'
#param_filename = '5R1P.txt'


# Attaching 3D axis to the figure
elev = 35
azim = 60
fig, ax = plt.subplots(subplot_kw=dict(projection="3d", elev=elev, azim=azim))

global robot
robot = readRobotParams(param_filename)

theta = np.zeros(len(robot.theta))
for i in linspace(0, 2*np.pi, 30):
    theta[0] = i

    x, y, z, u, v, w, colors = getJointAxes(theta)

    plt.cla()
    initPlot()
    for i in np.arange(0,len(x)):
        ax.quiver(x[i], y[i], z[i], u[i], v[i], w[i],
            color=colors[i], length=0.1, normalize=True)

    for i in range(len(robot.a)):
        def transformTube(Xc,Yc,Zc, T):
            Xc = Xc.flatten()
            Yc = Yc.flatten()
            Zc = Zc.flatten()
            ones = np.ones(len(Xc))
            tube = np.vstack([Xc, Yc, Zc, ones])

            T

            Xc = tube[0,:]
            Yc = tube[1,:]
            Zc = tube[2,:]

            r = np.sqrt(len(Xc))
            sz = (r, r)

            Xc = np.reshape(Xc,sz)
            Yc = np.reshape(Yc,sz)
            Zc = np.reshape(Zc,sz)

            return Xc,Yc,Zc

        Xc,Yc,Zc = data_for_cylinder_along_z(i/10,0.,0.04,abs(robot.a[i]))
        
        ax.plot_surface(Xc, Yc, Zc, alpha=0.5)
        
        Xc,Yc,Zc = data_for_cylinder_along_z(0,i/10,0.04,abs(robot.d[i]))

        ax.plot_surface(Xc, Yc, Zc, alpha=0.5)

        
        

        fwd_kin = forwardKinematics(robot)
        pp.pprint(np.around(fwd_kin[0],3))

    plt.pause(0.03)


plt.show()