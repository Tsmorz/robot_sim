from numpy import linspace, sin, cos, pi
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d
import os

import pprint
pp = pprint.PrettyPrinter(indent=4)

# Robot class that saves DH parameters
class Robot:
    def __init__(self, name, joint, theta, a, d, alpha):
        self.name = name
        self.joint = joint
        self.theta = theta
        self.a = a
        self.d = d
        self.alpha = alpha


# Save the DH parameters from a text file
def readRobotParams(param_filename):
    # Find text file with robot parameters
    cwd = os.getcwd()
    filename = cwd + '/robot_sim/' + param_filename

    joint = []
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
                joint.append(str(params[0]))
                theta.append(float(params[1]))
                a.append(float(params[2]))
                d.append(float(params[3]))
                alpha.append(float(params[4]))
            lines+=1

    robot = Robot(name, joint, theta, a, d, alpha)

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


def initPlot():
    ax.set_title(robot.name)

    # Setting the axes properties
    ax.set_xlim3d([-limit, limit])
    ax.set_xlabel('X (m)')
    ax.set_ylim3d([-limit, limit])
    ax.set_ylabel('Y (m)')
    ax.set_zlim3d([-limit, limit])
    ax.set_zlabel('Z (m)')


# plot each joint axes
def plotAxes(show_path):
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

    x, y, z, u, v, w, colors = getJointAxes(robot.theta)

    for i in np.arange(0,len(x)):
        if i<=2:
            ax.quiver(x[i], y[i], z[i], u[i], v[i], w[i],
                color=colors[i], length=0.2, normalize=True)
        else:
            if show_path:
                if i>=len(x)-3:
                    ax.quiver(x[i], y[i], z[i], u[i], v[i], w[i],
                        color=colors[i], length=0.1, normalize=True)
            else:
                ax.quiver(x[i], y[i], z[i], u[i], v[i], w[i],
                    color=colors[i], length=0.1, normalize=True)


# plot each link
def plotLinks():

    # create robot links
    def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
        z = np.linspace(0, height_z, 10)
        theta = np.linspace(0, 2*np.pi, 10)
        theta_grid, z_grid=np.meshgrid(theta, z)
        x_grid = radius*np.cos(theta_grid) + center_x
        y_grid = radius*np.sin(theta_grid) + center_y

        return x_grid,y_grid,z_grid

    # tranform links
    def transformTube(Xc,Yc,Zc, transform):
            Xc = Xc.flatten()
            Yc = Yc.flatten()
            Zc = Zc.flatten()
            ones = np.ones(len(Xc))
            tube = np.vstack([Xc, Yc, Zc, ones])

            tube = np.matmul(transform,tube)

            Xc = tube[0,:]
            Yc = tube[1,:]
            Zc = tube[2,:]

            r = int(np.sqrt(len(Xc)))

            Xc = np.reshape(Xc,[r,r])
            Yc = np.reshape(Yc,[r,r])
            Zc = np.reshape(Zc,[r,r])

            return Xc,Yc,Zc

    
    fwd_kin = forwardKinematics(robot)

    # loop through each link
    for i in range(len(robot.a)):
        if i==0:
            fwd_kin_chain = np.array(fwd_kin[i])
            
        else:
            fwd_kin_chain = np.matmul(fwd_kin_chain, fwd_kin[i])

        if abs(robot.d[i]) > abs(robot.a[i]):
            if robot.alpha[i] != 0:
                height_z = robot.d[i]
                T = [
                    [1,0,0,0],
                    [0,cos(robot.alpha[i]),-sin(robot.alpha[i]),0],
                    [0,sin(robot.alpha[i]),cos(robot.alpha[i]),0],
                    [0,0,0,1]
                ]
            else:
                height_z = -robot.d[i]
                T = np.eye(4)
        else:
            if robot.alpha[i] == 0:
                height_z = robot.a[i]
                T = [
                    [0,0,-1,0],
                    [0,1,0,0],
                    [1,0,0,0],
                    [0,0,0,1]
                ]
            else:
                height_z = robot.a[i]
                T = np.eye(4)

        transform = np.matmul(fwd_kin_chain, T)

        Xc,Yc,Zc = data_for_cylinder_along_z(0,0,0.04,height_z)
        Xc,Yc,Zc = transformTube(Xc,Yc,Zc,transform)
        ax.plot_surface(Xc, Yc, Zc, alpha=0.5)


# set robot joint positions
def setJointPosition(theta):
    for i in range(len(robot.theta)):
        robot.theta[i] = theta[i]


def animateJoints(theta, show_path):
    setJointPosition(theta)

    if show_path:
        plotAxes(show_path)
    else:
        plt.cla()
        initPlot()
        plotAxes(show_path)
        plotLinks()
        plt.pause(0.01)


def simplePath(start, stop, interval, show_path):
    joints = []
    for i in range(len(robot.theta)):

        q = linspace(start[i], stop[i], interval)
        if i==0:
            thetas = q
        else:
            thetas = np.vstack([thetas, q])

    for i in range(interval):
        theta = thetas[:,i]
        animateJoints(theta, show_path)


# ----- Main function --------------------------------
param_filename = '3R.txt'
param_filename = 'ur16e.txt'
#param_filename = 'ur5e.txt'
#param_filename = '5R1P.txt'
#param_filename = 'quad.txt'


# Attaching 3D axis to the figure
elev = 20
azim = 60
fig, ax = plt.subplots(subplot_kw=dict(projection="3d", elev=elev, azim=azim))

global robot
filename = 'robot models/' + param_filename
robot = readRobotParams(filename)

global limit
limit = 0.8
initPlot()

origin = np.zeros(len(robot.theta))
home = [0, 0, 0, 0, 0, 0]

start = home
stop = [0, 1.8*pi, -0.5*pi, 0, 0, 0]
stop = -2*pi*np.ones(6)
#stop[0] = 0
interval = 200

show_path = True
simplePath(start, stop, interval, show_path)

'''
simplePath(stop, start, interval, show_path)
simplePath(start, stop, interval, show_path)
simplePath(stop, start, interval, show_path)
simplePath(start, stop, interval, show_path)
simplePath(stop, start, interval, show_path)
'''
'''
for i in range(len(robot.theta)):
    stop[i] = 2*pi
    simplePath(start, stop, interval, show_path=False)
    #simplePath(stop, start, interval, show_path=False)
    stop[i] = home[i]
'''
plt.show()

#pp.pprint(np.around(fwd_kin[i],3))