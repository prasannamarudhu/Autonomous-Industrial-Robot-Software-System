echo "Starting Gazebo environment" 
xterm -hold -e "roslaunch group1_final_project ariac_manager.launch" & 

sleep 5
echo "Starting moveit for arm1 & arm2" 
xterm -hold -e "roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1" & 
xterm -hold -e "roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2" & 

sleep 10
echo "Starting main node" 
xterm -hold -e "rosrun group1_final_project main_node" & 
