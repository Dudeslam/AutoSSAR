# AutoSSAR
Master Thesis in Swarm intelligence for Search and Rescue

Before you run this algorithm, you need to install some dependencies:
Run the following to install dependencies:

>  sudo apt-get install libarmadillo-dev ros-melodic-nlopt 
>  sudo apt-get install libdw


To run the algorithm:
First change directory to the catkin workspace
'''console
cd ~\(your_directory_placement)\catkin_ws
'''

next make the program running:
'''console
catkin_make
'''

you should now see a build and a devel folder created in your workspace.
You can now run the program using:
'''console
source devel/setup.bash && roslaunch exploration_manager rviz_myMulti.launch
'''
to launch the Rviz tool for visualizing


'''console
source devel/setup.bash && roslaunch exploration_manager exploration_DOUBLE.launch
'''
for the launch of two UAVs

'''console
source devel/setup.bash && roslaunch coordination coordination_DOUBLE.launch
'''
to launch the coordination algorithm along with the mapmerge
