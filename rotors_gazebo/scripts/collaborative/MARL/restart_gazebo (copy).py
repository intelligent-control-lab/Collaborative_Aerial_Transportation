#!/usr/bin/env python3
import os
import time
import sys 
import subprocess, signal
import rosparam
import start_training1 as training
import rospy


def kill_collaborative_hovering():
    popen_outputs = os.popen('ps -ef | grep python').read()
    for line in popen_outputs.splitlines():
        if 'collaborative_hovering' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
        if 'joint_state_publisher'  in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
        if 'rosmaster' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
        if 'gzserver' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
        if 'gzclient' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))

            
    print "killing collaborative_hovering finished, sleep for 20 seconds..."
    time.sleep(20)

def kill_UAV_training():
    popen_outputs = os.popen('ps -ef | grep python').read()
    for line in popen_outputs.splitlines():
        if 'UAV_training' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
    print "killing UAV_training finished, sleep for 5 seconds..."
    time.sleep(5)

def launch():
    bash_cmd = "~/.bashrc"
    source_cmd = "source /home/lucasyu/catkin_ws/devel/setup.bash"

    str_launch_hovering = 'roslaunch rotors_gazebo collaborative_hovering.launch'
    str_launch_training = 'roslaunch rotors_gazebo UAV_training.launch'

    complete_cmd_collaborative_hovering = "gnome-terminal -e %s -e '%s' --tab -e '%s'"\
        % (bash_cmd, source_cmd, str_launch_hovering)

    # complete_cmd_UAVtraining = "gnome-terminal -e %s -e '%s' --tab -e '%s'"\
    #     % (bash_cmd, source_cmd, str_launch_training)

    os.system("%s" %(complete_cmd_collaborative_hovering))
    print 'launching collaborative_hovering node... sleep 10 seconds'
    time.sleep(10)
    # os.system("%s" %(complete_cmd_UAVtraining))
    # print 'launching UAVtraining node... sleep 15 seconds'
    # time.sleep(5)


if __name__ == '__main__':
    launch()

    while True:
        rospy.init_node('multi_UAV_gym', anonymous=False, log_level=rospy.INFO)
        flag = training.UAV_training()
        if flag:
            time.sleep(2)
            break
    for i in range(10):
        kill_collaborative_hovering()
        is_running = True
        while is_running:
            popen_outputs = os.popen('ps -ef | grep python').read()
            for line in popen_outputs.splitlines():
                if 'collaborative_hovering' in line:
                    print "collaborative_hovering is still running!"
                elif 'UAV_training' in line:
                    print "UAV_training is still running!"
            is_running = False

        launch()
        while True:
            rospy.init_node('multi_UAV_gym', anonymous=False, log_level=rospy.INFO)
            if training.UAV_training():
                time.sleep(2)
                break
    
    kill_collaborative_hovering()

        
        
    
