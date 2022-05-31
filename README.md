# Robotic Arm Simulator
This is a lightweight simulator to visualize robot paths. The program loads Denavit-Hartenberg parameters from a txt file to import the correct robot configuration. It's possible to view all joint axes or just the axes at the tool tip. If viewing all the axes, the program is slow so it's best to simply view the path after completion.

To-Do:
write functions to:
- calculate the Jacobian
- implement inverse-kinematics
- sample-based path planning like RRT and PRM
- implete a gripper for the tip of the robot
- visualizing robot path with a line instead of axes
- implement joint limits

<p float="left">
  <img src=https://user-images.githubusercontent.com/83112082/171044843-b050536b-c731-43d7-847c-193f58297eab.gif width="45%" height="45%" />
  <img src=https://user-images.githubusercontent.com/83112082/171084326-3e70696f-baf2-486a-94fe-0553d8db440e.gif width="45%" height="45%" />
</p>

## Denavit-Hartenberg Parameters
This is the format for the text file for the DH parameters. For joint type, R stands for revolute and P stands for prismatic.
| Robot_Name |     |      |      |     |
| ------------------- | :--- | :--- | :--- | :--- |
| joint | theta (rad) | a (m) | d (m)  | alpha (rad) |
| R     | 0           | 0     | 0.1807 | 1.5708      |
| R     | 0           | 0     | 0.1807 | 1.5708      |
| R     | 0           | 0     | 0.1807 | 1.5708      |
| R     | 0           | 0     | 0.1807 | 1.5708      |
