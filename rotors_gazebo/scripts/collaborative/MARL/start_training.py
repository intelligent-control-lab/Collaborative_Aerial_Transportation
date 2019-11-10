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
import multi_UAV_env
import os
import tensorflow as tf
import matplotlib.pyplot as plt
import sys
# from maddpg import MaDDPG
import subprocess, signal
# import os.system
import restart_gazebo as restarter


params = {   'num_episodes'   :  60000,
             'num_steps'      :  2000,
             'lr'             :  1e-2,
             'gamma'          :  0.95,
             'batch_size'     :  1024,
             'num_units'      :  256  ,
             'exp_name'       :  None,
             'save_dir'       :  "/home/awesomelb/Multi-UAV_RL/src/Collaborative_Aerial_Transportation/collaborative_training/src/policy/",
             'save_rate'      :  100,
             'load_dir'       :  "",
             'restore'        :  False,
             'display'        :  False,
             'benchmark'      :  False,
             'benchmark_iters': 100000,
             'benchmark_dir'  : "./benchmark_files/",
             'plots_dir'      : "./learning_curve/"
                 
                 }



if __name__ == '__main__':
    
    rospy.init_node('multi_UAV_gym', anonymous=True, log_level=rospy.INFO)
    # restarter.launch()
    # Create the Gym environment
    env = gym.make('multi_UAV-v0')
    #obs_shape_n = [env.observation_space[i].shape for i in range(4)]

    #trainers = get_trainers(env, obs_shape_n, params)
    state_dim = 20
    action_dim = 3
    num_agents = 4

#####################################################################
    maddpg = MaDDPG(num_agents,state_dim, action_dim)
#########################################################

    

    nepisodes = 20
    nsteps = 6
    print ("start ...")
    
    for i in range(nepisodes):

        print "episode: ", i+1
        # Initialize the environment and get first state of the robot
        print "env.reset..."
        # Now We return directly the stringuified observations called state

        current_state = env.reset()

        positions = [[0.0, 0.0, 0.0],
                     [2.0, 0.0, 2.0],
                     [4.0, 0.0, 4.0]]
        velocities = [[0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0]]
        env.compute_trj(positions, velocities)
        env.gazebo.unpauseSim()
        time.sleep(2.0)
        count = 0
        
        flag = env.hover_and_load_trj()

        # count = count+1
        # print "count:", count
        # if count ==4:
            # break
        
        env.gazebo.pauseSim()

        current_state = env.get_step_state()
        print current_state

        for j in range(nsteps):

            print "step: ", j
            actions = maddpg.noise_action(current_state)
            # actions = [np.ones((3, 1)) * 0.25 for _ in range(4)]
            actions = np.reshape(np.array(actions),(1,-1))[0]
            print "actions: ", actions
            # env.update_c_from_action(actions)
            next_state, reward, done = env.step(actions)
            maddpg.perceive(current_state,action,reward,next_state,done)
            current_state = next_state
            if done:
                print('Done!!!!!!!!!!!! at epoch{} , reward:{}'.format(epoch,reward))
            #add summary for each episode
                maddpg.summary(episode)
                break

        # p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
        # out, err = p.communicate()
        # for line in out.splitlines():
        #     if 'gzserver' in line:
        #         pid = int(line.split(None, 1)[0])
        #         os.kill(pid, signal.SIGKILL)
            
        #os.system('killall -9 rosmaster')
        env.gazebo.unpauseSim()
        # time.sleep(2.0)
        # launch()
    #env.close()


'''
for episode in range(max_episode):
    print('episode',episode)
    #while (True):
        #Env.re_create_env(num_agents)
    current_state = Env.reset()
        #action = maddpg.noise_action(current_state)
        #next_state, reward, done = Env.step(action)
        #print(reward)
       # if not done:
       #    current_state = next_state
       #      break

    for epoch in range(max_epoch):
        #print('epoch',epoch)
        #Env.render()
        positions = [[0.0, 0.0, 0.0],
                     [2.0, 0.0, 2.0],
                     [4.0, 0.0, 4.0]]
        velocities = [[0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0]]

        env.compute_trj(positions, velocities)
        env.hover_and_load_trj()

        action = maddpg.noise_action(current_state)
        env.update_c_from_action()
        #print(action)
        next_state, reward, done = Env.step(action)
        maddpg.perceive(current_state,action,reward,next_state,done)
        current_state = next_state
        if done:
            print('Done!!!!!!!!!!!! at epoch{} , reward:{}'.format(epoch,reward))
            # add summary for each episode
            maddpg.summary(episode)
            break
    if epoch ==max_epoch-1:
        print('Time up >>>>>>>>>>>>>>')
        '''




    
