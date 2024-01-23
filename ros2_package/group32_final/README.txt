README


==================================================================================================================================================================
==================================================================================================================================================================
1. We have created a video recording of our simulation running, and store it in this following link. Please see for your additional reference.

Please see the link below for the simulation recording.

https://drive.google.com/file/d/188-EA3kCCvZj-4ychJ1gfWo_ZM_F6vQV/view?usp=sharing

==================================================================================================================================================================
==================================================================================================================================================================

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


