#!/usr/bin/env python3
import os
import time
import sys 
import subprocess, signal
import rosparam
import rospy
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

done_file = '/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/maddpg_training/done.txt'


def kill_collaborative_hovering():
    popen_outputs = os.popen('ps -ef | grep python').read()
    for line in popen_outputs.splitlines():
        if 'collaborative_hovering' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
        if 'joint_state_publisher'  in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
        # if 'rosmaster' in line:
        #     pid = line.split()[1]
        #     os.system("kill %s"%(pid))
        if 'gzserver' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
        if 'gzserver' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))

    time.sleep(1)        
    print "killing collaborative_hovering finished, sleep for 20 seconds..."


def kill_UAV_training():
    popen_outputs = os.popen('ps -ef | grep python').read()
    for line in popen_outputs.splitlines():
        if 'UAV_training' in line:
            pid = line.split()[1]
            os.system("kill %s"%(pid))
    print "killing UAV_training finished, sleep for 5 seconds..."
    time.sleep(1)

def launch():
    bash_cmd = "~/.bashrc"
    source_cmd = "source /home/lucasyu/catkin_ws/devel/setup.bash"

    str_launch_hovering = 'roslaunch rotors_gazebo collaborative_hovering.launch'
    str_launch_training = 'roslaunch rotors_gazebo UAV_training.launch'

    complete_cmd_collaborative_hovering = "gnome-terminal -e %s -e '%s' --tab -e '%s'"\
        % (bash_cmd, source_cmd, str_launch_hovering)

    complete_cmd_UAVtraining = "gnome-terminal -e %s -e '%s' --tab -e '%s'"\
        % (bash_cmd, source_cmd, str_launch_training)

    os.system("%s" %(complete_cmd_collaborative_hovering))
    print 'launching collaborative_hovering node... sleep 15 seconds'
    time.sleep(12)
    os.system("%s" %(complete_cmd_UAVtraining))
    print 'launching UAVtraining node...'
    

rosout_old = None
def callback_joint_states(msg):
    # print msg.header
    global rosout_old
    rosout_old = msg.data
    # print rosout_old


if __name__ == '__main__':
    core_dumped = False
    source_cmd = "source /home/lucasyu/catkin_ws/devel/setup.bash"
    os.system("gnome-terminal -e '%s' -e roscore" %(source_cmd))

    rospy.init_node('monitoring_joint_states', anonymous=False, log_level=rospy.INFO)
    cb_js = rospy.Subscriber('/hummingbird_3/motor_speed/0', Float32, callback_joint_states)

    # source_cmd = "source /home/lucasyu/catkin_ws/devel/setup.bash"
    # monitoring_cmd = "python /home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/test_wait.py"
    # os.system("gnome-terminal -e '%s' --tab -e roscore --tab -e '%s'" %(source_cmd, monitoring_cmd))

    # launch()
    # while True:
    #     rospy.init_node('multi_UAV_gym', anonymous=False, log_level=rospy.INFO)
    #     flag = training.UAV_training()
    #     if flag:
    #         time.sleep(2)
    #         break
    nepisodes =  10000
    success_episodes = 0
    start_episodes = time.time()
    for i in range(nepisodes):
        print "episode: ", i+1
        FLAG_1 = 1
        FLAG_2 = 1
        FLAG_3 = 1
        FLAG_4 = 1
        FLAG_5 = 1
        FLAG_6 = 1
        while True:
            popen_outputs = os.popen('ps -ef | grep python').read()
            for line in popen_outputs.splitlines():
                if 'collaborative_hovering' not in line:
                    FLAG_1 = 0
                    # print "collaborative_hovering not running!"
                if 'UAV_training' not in line:
                    FLAG_2 = 0
                    # print "UAV_training not running!"
                if 'joint_state_publisher' not in line:
                    FLAG_3 = 0
                    # print "joint_state_publisher not running!"
                if 'rosmaster' not in line:
                    FLAG_4 = 0
                    # print "rosmaster not running!"
                if 'gzserver' not in line:
                    FLAG_5 = 0
                    # print "gzserver not running!"
                if 'gzserver' not in line:
                    FLAG_6 = 0
                    # print "gzserver not running!"
            if FLAG_1 == 0 and FLAG_2 == 0 and FLAG_3 == 0 and FLAG_4 == 0 and FLAG_5 == 0 and FLAG_6 == 0:
                print "process clean, no related ones are running, sleep for 15 second..."
                break
            else:
                print "some gazebo process is running... "
        
        if core_dumped:
            core_dumped = False
            time.sleep(7)
        else:
            time.sleep(13)
        start = time.time()
        launch()
        data_flag = None
        while True:
            f_done = open(done_file,'r+')
            done = f_done.read(1)
            f_done.close()
            end = time.time()
            time_spend = end - start

            # core dumped when the models are prepared
            if time_spend > 20.0 and time_spend < 21.0:
                print "time: ",time_spend, " rosout_old: ",  rosout_old
                if rosout_old is None:
                    core_dumped = True
                    print "core dumped... kill"
                    f_done = open(done_file,'w')
                    f_done.write('1')
                    f_done.close()
                else:
                    rosout_old = None
                    time.sleep(0.2)
                    print "rosout_old after made None: ", rosout_old

            # core dumped when the controller are prepared
            if time_spend > 35.0 and time_spend < 36.5:
                print "time: ",time_spend, " rosout_old: ",  rosout_old
                if not rosout_old is None:
                    rosout_old = None
                    time.sleep(1.0)
                    print "rosout_old after made None: ", rosout_old
                print rosout_old
                if rosout_old is None:
                    core_dumped = True
                    print "core dumped... kill"
                    f_done = open(done_file,'w')
                    f_done.write('1')
                    f_done.close()
                
            # if time_spend > 10.0:
            #     try:
            #         data_flag = rospy.wait_for_message("/rosout", Log, timeout=5.0) 

            #     except rospy.exceptions.ROSException:
            #         print data_flag
            #         print "core dumped... kill"
            #         f_done = open(done_file,'w')
            #         f_done.write('1')
            #         f_done.close()
                    
            if (done != '') or (time_spend > 200.0):
                f_done = open(done_file,'w')
                f_done.close()  
                success_episodes = success_episodes + 1
                end_episode = time.time()
                print "time average spend on episodes: ", (end_episode - start_episodes) / success_episodes
                break
        time.sleep(0.5)
        kill_collaborative_hovering()
        kill_UAV_training()
        
        print "time spend: %f"%(time_spend)
        # while True:
        #     rospy.init_node('multi_UAV_gym', anonymous=False, log_level=rospy.INFO)
        #     if training.UAV_training():
        #         time.sleep(2)
        #         break
    
    kill_collaborative_hovering()
    kill_UAV_training()

        
        
    
