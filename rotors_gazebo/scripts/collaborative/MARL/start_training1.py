#!/usr/bin/env python

'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Moded by Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com

'''
import gym
import time
import numpy as np
import random
# import tensorflow.contrib.layers as layers

from gym import wrappers
from std_msgs.msg import Float64

# ROS packages required
import rospy
import rospkg
from gazebo_connection import GazeboConnection
# import our training environment
from maddpg import MaDDPG
import multi_UAV_env
import os
import tensorflow as tf
import matplotlib.pyplot as plt
import sys
from nav_msgs.msg import Odometry
import tf
from math import *

done_file        = '/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/maddpg_training/done.txt'
reward_file      = '/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/maddpg_training/reward.txt'
reward_test_file      = '/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/maddpg_training/reward_test.txt'


params = {   'num_episodes'   :  60000,
             'num_steps'      :  2000,
             'lr'             :  1e-2,
             'gamma'          :  0.95,
             'batch_size'     :  1024,
             'num_units'      :  256  ,
             'exp_name'       :  None,
             'save_dir'       :  "/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/maddpg_training/policy/",
             'save_rate'      :  100,
             'load_dir'       :  "",
             'restore'        :  False,
             'display'        :  False,
             'benchmark'      :  False,
             'benchmark_iters': 100000,
             'benchmark_dir'  : "./benchmark_files/",
             'plots_dir'      : "./learning_curve/"
                 
                 }


def quaternion2euler(quaternion_list):
    return tf.transformations.euler_from_quaternion(quaternion_list)

payload_euler_angle = np.zeros((3, 1))
payload_height = 0.0

def callback_payload_odometry(data):
  global payload_euler_angle
  global payload_height
  payload_euler_angle = np.array(quaternion2euler([data.pose.pose.orientation.x,
                                                     data.pose.pose.orientation.y,
                                                     data.pose.pose.orientation.z,
                                                     data.pose.pose.orientation.w])).reshape((3, 1))
  payload_height = data.pose.pose.position.z
  if system_destroyed():
    print "System Destroyed!!"

def system_destroyed():
  disfunction_height = 1.0
  if (fabs(payload_euler_angle[0, 0]) > 1.0) or (fabs(payload_euler_angle[1, 0]) > 1.0) or (payload_height < disfunction_height):
    return True
  else:
    return False

if __name__ == '__main__':
    state_dim = 20
    action_dim = 3
    num_agents = 4
    maddpg = MaDDPG(num_agents,state_dim, action_dim)
    rospy.init_node('multi_UAV_gym', anonymous=False, log_level=rospy.INFO)
    env = gym.make('multi_UAV-v0')

    topic_name_odom = '/payload/ground_truth/odometry'
    sub_payload_odometry = rospy.Subscriber(topic_name_odom, Odometry, callback_payload_odometry)
    #obs_shape_n = [env.observation_space[i].shape for i in range(4)]
    maddpg.load_network()
    #trainers = get_trainers(env, obs_shape_n, params)

    f_reward = open(reward_file,'r+')
    episode = 0
    for line in f_reward.readlines():
        episode = episode +1
    f_reward.close()
#####################################################################


#########################################################
    nsteps = 600
    print ("start ...")

    # current_state = env.reset()

    positions = [[0.0, 0.0, 0.0],
                 [2.0, 0.0, 2.0],
                 [4.0, 0.0, 4.0]]
    velocities = [[0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0]]
    position_array = np.array(positions)
    final_goal_position = position_array[-1, :].reshape((3, 1))
    # env.set_goal_position(final_goal_position)
    env.compute_trj(positions, velocities)
    env.gazebo.unpauseSim()
    time.sleep(2.0)
    count = 0
    
    flag = env.hover_and_load_trj()
    
    env.gazebo.pauseSim()

    current_state = env.get_step_state()
    #print current_state
    
    reward_total = 0
    reward_total_test = 0
    for j in range(nsteps):

        print "step: ", j
        # if episode % 10 == 0 and episode != 0:
        actions = maddpg.action(current_state)
        # else:
            # actions = maddpg.noise_action(current_state)
        actions_bounded = np.clip(actions, 0, 1)
        #actions = [np.ones((3, 1)) * 0.25 for _ in range(4)]
        actions_c = np.reshape(np.array(actions_bounded),(1,-1))[0]
        #print "actions: ", actions
        # env.update_c_from_action(actions)
        # print(actions_c)
        next_state, reward = env.step(actions_c)
        done = system_destroyed()
        if done:
          reward[0] = reward[0] - 2000
        #print next_state,reward,done
        if episode % 10 == 0 and episode != 0:
            reward_total_test = reward_total_test + reward[0]
        reward_total = reward_total + reward[0]
        maddpg.perceive(current_state,actions_bounded,reward,next_state,done)
        current_state = next_state
        if done or j== nsteps-1 :
            print "final_goal_position: ", final_goal_position
            reward_arrive = env.compute_reward_arrive()
            reward_total = reward_total + reward_arrive
            print('Done!!!!!!!!!!!! at reward:{}'.format(reward_total))
            f_reward = open(reward_file,'a+')
            f_reward.write(str(float(reward_total)))
            f_reward.write('\n')
            f_reward.close()
            if episode % 10 == 0 and episode != 0:
                f_reward_test = f_reward_test + reward_arrive
                f_reward_test = open(reward_test_file,'a+')
                f_reward_test.write(str(float(reward_total_test)))
                f_reward_test.write('\n')
                f_reward_test.close()
            #add summary for each episode
            maddpg.summary(episode)
            break
    
    env.show_accumulated_force()
    f_done = open(done_file,'w')
    f_done.write('1')
    f_done.close()
    #env.mellingers = []

    # env.close()
    # return True





    
