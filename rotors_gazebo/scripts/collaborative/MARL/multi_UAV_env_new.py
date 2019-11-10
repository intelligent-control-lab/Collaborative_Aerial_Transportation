#!/usr/bin/env python
'''
    By Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import gym
import rospy
import numpy as np
import time

from gym import utils, spaces

from geometry_msgs.msg import Pose

from gym.utils import seeding

from gym.envs.registration import register

from gazebo_connection import GazeboConnection

from multi_UAV_state import MultiUavState

from controllers_connection import ControllersConnection

from mellinger_MARL import MellingerMARL

from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState

from gazebo_msgs.srv import DeleteModel, SpawnModelRequest, SpawnModel

import os

#register the training environment in the gym as an available one
reg = register(
    id='multi_UAV-v0',
    entry_point='multi_UAV_env:multi_UAV_Env',
    timestep_limit=50,
    )


class multi_UAV_Env(gym.Env):

    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment
        self.set_link = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        rospy.wait_for_service('/gazebo/set_link_state')
        # gets training parameters from param server
        self.desired_pose = Pose()
        self.desired_pose.position.x = rospy.get_param("/desired_pose/position/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/position/y")
        self.desired_pose.position.z = rospy.get_param("/desired_pose/position/z")
        self.desired_pose.orientation.x = rospy.get_param("/desired_pose/orientation/x")
        self.desired_pose.orientation.y = rospy.get_param("/desired_pose/orientation/y")
        self.desired_pose.orientation.z = rospy.get_param("/desired_pose/orientation/z")

        self.running_step = rospy.get_param("/running_step")
        self.max_incl = rospy.get_param("/max_incl")
        self.max_vel = rospy.get_param("/max_vel")
        self.min_vel = rospy.get_param("/min_vel")
        self.max_acc = rospy.get_param("/max_acc")
        self.min_acc = rospy.get_param("/min_acc")
        self.max_jerk = rospy.get_param("/max_jerk")
        self.min_jerk = rospy.get_param("/min_jerk")
        self.max_snap = rospy.get_param("/max_snap")
        self.min_snap = rospy.get_param("/min_snap")

        self.done_reward = rospy.get_param("/done_reward")
        self.alive_reward = rospy.get_param("/alive_reward")


        self.num_action_space = 15

        high = np.array([1. for _ in range(self.num_action_space)])
        low = np.array([-1. for _ in range(self.num_action_space)])
        self.action_space = spaces.Box(low=low, high=high)


        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.controllers_object = ControllersConnection(namespace="monoped")

        self.multi_uav_state_object = MultiUavState(  max_vel=self.max_vel, 
                                                    min_vel=self.min_vel, 
                                                    max_acc=self.max_acc,
                                                    min_acc=self.min_acc,
                                                    max_jerk=self.max_jerk,
                                                    min_jerk=self.min_jerk,
                                                    max_snap=self.max_snap,
                                                    min_snap=self.min_snap, 
                                                    abs_max_roll=self.max_incl,
                                                    abs_max_pitch=self.max_incl,
                                                    done_reward=self.done_reward,
                                                    alive_reward=self.alive_reward
                                                )

        self.multi_uav_state_object.set_desired_world_point(self.desired_pose.position.x,
                                                              self.desired_pose.position.y,
                                                              self.desired_pose.position.z,
                                                              self.desired_pose.orientation.x,
                                                              self.desired_pose.orientation.y,
                                                              self.desired_pose.orientation.z
                                                          )
        
        self.mellingers = [
                      MellingerMARL('hummingbird', 0, 0.95, 0.0, 0.35),
                      MellingerMARL('hummingbird', 1, 0.0, 0.95, 0.35),
                      MellingerMARL('hummingbird', 2, -0.95, 0.0, 0.35),
                      MellingerMARL('hummingbird', 3, 0.0, -0.95, 0.35)
                      ]


        """
        For this version, we consider 6 actions
        1-2) Increment/Decrement haa_joint
        3-4) Increment/Decrement hfe_joint
        5-6) Increment/Decrement kfe_joint
        """
        self.action_space = spaces.Discrete(6)
        self.reward_range = (-np.inf, np.inf)

        self._seed()
        print ("end of init...")

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def _reset(self):
        # 0st: We pause the Simulator
        print("Pausing SIM...")
        self.gazebo.pauseSim()

        # 1st: resets the simulation to initial values
        print("Reset SIM...")
        self.gazebo.resetSim()
        self.gazebo.unpauseSim()

        # model_name = "four_quad_payload"
        # rospy.wait_for_service('/gazebo/delete_model')
        # del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        # del_model_prox(model_name)


        # model_xml = rospy.get_param("robot_description")
        # pose = Pose()
        # pose.orientation.w = 1.0
        
        # spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        # req = SpawnModelRequest()
        # req.model_name = model_name
        # req.model_xml = model_xml
        # # should this be unique so ros_control can use each individually?
        # req.robot_namespace = "/foo"
        # req.initial_pose = pose
        # resp = spawn_model(req)

        self.gazebo.resetWorld()

        # os.system('gnome-terminal -x roslaunch rotors_gazebo spawn_quad_sphere_load.launch')

        # initial_pose = Pose()
        # initial_pose.position.x = 0
        # initial_pose.position.y = 0
        # initial_pose.position.z = 0

        # f = open('/home/icl-baby/catkin_ws/src/Collaborative_Aerial_Transportation/rotors_description/urdf/collaborative/four_hummingbirds_payload.xacro')
        # urdf = f.read()
        # rospy.wait_for_service('gazebo/spawn_urdf_model')
        # spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        # spawn_model_prox("four_quad_payload", urdf, "hummingbird", initial_pose, "world")
        
        # for i in range(4):
        #     rospy.wait_for_service('/gazebo/set_link_state')
        #     link_imu_name = 'hummingbird_' + str(i) + '/imugt_link'
        #     self.set_link(LinkState(link_name=link_imu_name))
        #     for j in range(4):
        #         rospy.wait_for_service('/gazebo/set_link_state')
        #         link_name = 'hummingbird_' + str(i) + '/rotor_' + str(j)
        #         print link_name
        #         self.set_link(LinkState(link_name=link_name))
        #     rospy.wait_for_service('/gazebo/set_link_state')    
        #     link_odom_name = 'hummingbird_' + str(i) + '/odometry_sensorgt_link'
        #     self.set_link(LinkState(link_name=link_odom_name))

        # 2nd: We Set the gravity to 0.0 so that we dont fall when reseting joints
        # It also UNPAUSES the simulation
        print("Remove Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.8)

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        # rospy.logdebug("reset_monoped_joint_controllers...")
        # self.controllers_object.reset_monoped_joint_controllers()

        # 3rd: resets the robot to initial conditions
        # print("set_init_pose...")
        # for controller in self.mellingers:
        #     motor_speed = np.zeros((4, 1))
        #     controller.set_desired_motor_speed(motor_speed)
            # controller.send_motor_command()

        # 5th: Check all subscribers work.
        # Get the state of the Robot defined by its RPY orientation, distance from
        # desired point, contact force and JointState of the three joints
        print("check_all_systems_ready...")
        self.multi_uav_state_object.check_all_systems_ready()
        # rospy.logdebug("get_observations...")
        # observation = self.multi_uav_state_object.get_states()

        # 6th: We restore the gravity to original
        print("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        # 7th: pauses simulation
        print("Pause SIM...")
        self.gazebo.pauseSim()

        # Get the State Discrete Stringuified version of the observations
        state = self.get_state()

        return state

    def _step(self, actions):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot

        # 1st, decide which action corresponsd to which joint is incremented

        # next_action_position = self.multi_uav_state_object.get_action_to_position(action)

        # update the reference in the mellinger controller from the action
        for i, action in enumerate(actions):
            self.mellingers[i].set_ref_from_action(action)

        # We move it to that pos
        self.gazebo.unpauseSim()
        # action to the gazebo environment
        for i_mellinger in self.mellingers:
            i_mellinger.publish_err()
            i_mellinger.update_current_state()
            i_mellinger.update_desired_values()
            i_mellinger.motorSpeedFromU()
            i_mellinger.send_motor_command()

        # self.monoped_joint_pubisher_object.move_joints(next_action_position)
        # Then we send the command to the robot and let it go
        # for running_step seconds
        time.sleep(self.running_step)
        self.gazebo.pauseSim()

        # We now process the latest data saved in the class state to calculate
        # the state and the rewards. This way we guarantee that they work
        # with the same exact data.
        # Generate State based on observations
        # observation = self.multi_uav_state_object.get_observations()

        # finally we get an evaluation based on what happened in the sim
        # reward,done = self.multi_uav_state_object.process_data()

        reward,done = self.multi_uav_state_object.calculate_reward_payload_orientation()

        # Get the State Discrete Stringuified version of the observations
        # state = self.get_state(observation)
        state = self.get_state()

        return state, reward, done

    def get_state(self):
        """
        We retrieve the Stringuified-Discrete version of the given observation
        :return: state
        """
        # return self.multi_uav_state_object.get_state_as_string(observation)

        return self.multi_uav_state_object.get_states()
