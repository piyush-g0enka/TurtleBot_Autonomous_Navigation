README

------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Steps to run the project
------------------------------------------------------------------------------------------------------------------------------------------------------------------

1. cd to the workspace, build the project "group32_final" with command 
	>_ colcon build --packages-select group32_final

2. Source the underlay

3. Launch the launch file from the turtlebot3_gazebo package with the following command
	>_ ros2 launch final_project final_project.launch.py

4. Wait for Gazebo to launch properly

5. Open another terminal, launch our custom launch file from the "group32_final" package with the following command
	>_ ros2 launch group32_final group32.launch.py
	
6. Now the robot should be moving, and relevant information will be printing out in the terminal.

===================================================================================================================================================================


