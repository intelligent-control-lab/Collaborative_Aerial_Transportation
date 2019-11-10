import os
import time
import sys 
base_save_path = os.getcwd()


def multilaunch(arg_type):

	bash_cmd = "~/.bashrc"
	source_cmd = "source /home/gg/intel_nav_ws/devel/setup.bash "
	cd_scripts_cmd = "cd ~/intel_nav_ws/src/EAI-E1/scripts/"
	kinect_cmd = "roslaunch kinect2_bridge kinect2_bridge.launch"
	joynode_cmd = "rosrun joy joy_node"
	eed_rgb_cmd = "roslaunch eed_display pub_lantern_and_display.launch display_type:=rgb"
	eed_depth_cmd = "roslaunch eed_display pub_lantern_and_display.launch display_type:=depth"
	eed_depth_semantic_cmd = "roslaunch eed_display pub_lantern_and_display.launch display_type:=depth_semantic"
	yolo_cmd = "roslaunch darknet_ros yolo_v3.launch"
	recorder_cmd = "rosrun eed_display recordRouteCommReal"
	dashgo_cmd = "python /home/gg/intel_nav_ws/src/EAI-E1/scripts/dashgo_driver.py"


	joy_control_cmd = "python /home/gg/intel_nav_ws/src/EAI-E1/scripts/joy_control.py"
	rgb_net_cmd = "python /home/gg/intel_nav_ws/src/EAI-E1/scripts/rgb_cnn_nornn_ros_real_scenario.py"
	depth_net_cmd = "python /home/gg/intel_nav_ws/src/EAI-E1/scripts/depth_cnn_nornn_ros_real_scenario.py"
	depth_semantic_net_cmd = "python /home/gg/intel_nav_ws/src/EAI-E1/scripts/depth_semantic_two_netsdepth_short_ros_black_bcg_real_scenario.py"
	rgb_segmented_net_cmd = "python /home/gg/intel_nav_ws/src/EAI-E1/scripts/segmented_cnn_nornn_ros_real_scenario.py"
	segmented_generate_cmd = "python /home/gg/intel_nav_ws/src/EAI-E1/scripts/rgb_segmentation_data_generation_ros.py"



	## assign the right type of nn 
	if arg_type == 'depth':
		complete_cmd_py = "gnome-terminal -e %s -e '%s' -e '%s'"\
	 	% (bash_cmd, source_cmd, depth_net_cmd)
	 	## assign the ros commands
		complete_cmd_ros = "gnome-terminal -e %s -e '%s' --tab -t 'kinect_cmd' -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s'"\
		% (bash_cmd, source_cmd, kinect_cmd, joynode_cmd, eed_depth_cmd, yolo_cmd, recorder_cmd, dashgo_cmd)
	elif arg_type == 'depth_semantic':
		complete_cmd_py = "gnome-terminal -e %s -e '%s' -e '%s'"\
	 	% (bash_cmd, source_cmd, depth_semantic_net_cmd)
		## assign the ros commands
		complete_cmd_ros = "gnome-terminal -e %s -e '%s' --tab -t 'kinect_cmd' -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s'"\
		% (bash_cmd, source_cmd, kinect_cmd, joynode_cmd, eed_depth_semantic_cmd, yolo_cmd, recorder_cmd, dashgo_cmd)
	elif arg_type == 'rgb':
		complete_cmd_py = "gnome-terminal -e %s -e '%s' -e '%s'"\
	 	% (bash_cmd, source_cmd, rgb_net_cmd)
	 	## assign the ros commands
		complete_cmd_ros = "gnome-terminal -e %s -e '%s' --tab -t 'kinect_cmd' -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s'"\
		% (bash_cmd, source_cmd, kinect_cmd, joynode_cmd, eed_rgb_cmd, yolo_cmd, recorder_cmd, dashgo_cmd)
	elif arg_type == 'segmented':
		complete_cmd_py = "gnome-terminal -e %s -e '%s' --tab -e '%s' --tab -e '%s'"\
	 	% (bash_cmd, source_cmd, rgb_segmented_net_cmd, segmented_generate_cmd)
		## assign the ros commands
		complete_cmd_ros = "gnome-terminal -e %s -e '%s' --tab -t 'kinect_cmd' -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s' --tab -e '%s'"\
		% (bash_cmd, source_cmd, kinect_cmd, joynode_cmd, eed_rgb_cmd, recorder_cmd, dashgo_cmd)
	else:
		print("wrong type name! must set a correct type to run: depth / depth_semantic / rgb")
		return

	

	os.system("%s" %(complete_cmd_ros))
	time.sleep(3)
	os.system("%s" %(complete_cmd_py))


if __name__ == '__main__':
	args = sys.argv[1:]
	multilaunch(args[0])