Cristian Oliveira, Theo Barnes-Cole, Joshua Lee
RBE 4540 Group Assignment 
Jani 
10/11/25

This is the README.txt file for our Group Assignment for RBE 4540 Vision Based Manipulation

In order to run our code and see if our project works, please follow the steps below.

Steps: 
1. Download and Unzip Final_Project.zip file
2. Make sure to have the following installed: python3, pip, pcl
3. Follow the project setup enviornment file on Canvas 
4. Colcon build the package with colcon build
5. Source the workspace with source install/setup.bash
6. Launch the Simulation with ros2 launch vbm_project_env simulation.launch.py
7. Open up rviz with rviz2 (or rviz with rviz)
8. Set frame to world abd add "By Topic" and select Topic 
9. In Gazebo, setup Test Scenario 1 (Red Sphere), or Test Scenario 2 (Gray Cube) according to Pictures Found in Final Report. 
*Note* The Red Sphere is an exisiting model in Gazebo, it is called Cricket Ball, the table is called Table, and is an existing model, and the Gray Cube is actually a URDF File called blue_cube.urdf, and can be spawned with the following command:
ros2 run gazebo_ros spawn_entity.py -file ~/Final_Project/RBE4540FinalProjectGroup1/project_ws/src/vbm_project_env/urdf/blue_cube.urdf -entity blue_cube 
**Note 2** Make sure to change the file directory in the command to where your URDF File is actually located 
***Note 3*** Use the Move button to move the object into place, usually we have to move the table down in the z axis after centering it underneath the camera, then turn off physics, then move the red sphere/gray cube underneath the table, then raise them in the z until they are above the table, and then turn physics back on
10. Once you have setup the Scenario in Gazebo, go to the project_ws folder in the terminal, colcon build, source install/setup.bash, then run the following command:
ros2 run vbm_project_env test1 
11. If done properly, the terminal should be saying Receiving Video Frame and spitting out the closest and opposite closest point, along with the grasp metric of that point. 

*Note*
 DO NOT USE OTHER OBJECTS AS WE ONLY TESTED WITH THESE 2 AND RESULTS MAY BE INCONSISTENT