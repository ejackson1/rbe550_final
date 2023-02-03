Running the simulation: roslaunch panda_simulation panda_simulation.launch
Installing potentially missing dependencies (run outside the src folder of the workspace): rosdep install --from-paths --ignore-src ./ -y

Note that the simulation starts paused to give the controllers time to connect to the robot, which prevents the robot from potentially falling over once the simulation starts

Tutorials:
C++: https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
Python: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
