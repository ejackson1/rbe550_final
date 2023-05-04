Requirements:
*RosNoetic
*Python3
Download all required dependancies

Running:
roslaunch panda_simulation panda_simulation.launch
rviz rviz : add planenr, and point cloud2, subscribe to outliers_2
rosrun panda_moveit_controller moveitServer.py
rosrun pcl centroid_grab_class
rosrun pcl stitching.py
rosrun panda_moveit_controller moveitClient.py

Bugs/Notes
* The Gazeebo dynamics make grasping somewhat unreliable and some poses cause the cube to fall