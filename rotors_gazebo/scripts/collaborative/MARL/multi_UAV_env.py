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
import sys
# from controllers_connection import ControllersConnection
sys.path.append("/home/awesomelb/Multi-UAV_RL/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL")
from mellinger_marl import Mellinger_Agent

from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState

from gazebo_msgs.srv import DeleteModel, SpawnModelRequest, SpawnModel
import rosparam
import os

from rosgraph_msgs.msg import Log

import time
#register the training environment in the gym as an available one
reg = register(
id='multi_UAV-v0',
entry_point='multi_UAV_env:multi_UAV_Env',
# timestep_limit=50,
)
done_file = '/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/maddpg_training/done.txt'

clock_flag = False

def callback_log(data):
    # print "callback_log"
    global clock_flag
    clock_flag = True

class multi_UAV_Env(gym.Env):

    def __init__(self):
        # os.system('rosparam load /home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/config/MADDPG_params.yaml')
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

        self.sub_clock = rospy.Subscriber('/rosout', Log, callback_log)

        self.num_agents = 4
        self.num_action_space = 12

        self.rate = rospy.Rate(50.0)

        high = np.array([1. for _ in range(self.num_action_space)])
        low = np.array([0. for _ in range(self.num_action_space)])
        self.action_space = spaces.Box(low=low, high=high)


        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        # self.controllers_object = ControllersConnection(namespace="monoped")

        self.multi_uav_state_object = MultiUavState(max_vel=self.max_vel, 
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
            Mellinger_Agent(
                mav_name='hummingbird',
                index=0,
                num_agents=4,
                c=0.4,
                x=0.95, y=0.0, z=0.35,
                dim=3
            ),
            Mellinger_Agent(
                'hummingbird',
                index=1,
                num_agents=4,
                c=0.1,
                x=0.0, y=0.95, z=0.35,
                dim=3
            ),
            Mellinger_Agent(
                'hummingbird',
                index=2,
                num_agents=4,
                c=0.15,
                x=-0.95, y=0.0, z=0.35,
                dim=3
            ),
            Mellinger_Agent(
                'hummingbird',
                index=3,
                num_agents=4,
                c=0.35,
                x=0.0, y=-0.95, z=0.35,
                dim=3
            )
        ]

        self.goal_position = np.zeros((3, 1))

        # print "clock_flag: ", clock_flag
        # timer_0 = rospy.Time.now()
        # while not clock_flag:
        #     print "No clock message received!"
        #     try:
        #         rospy.wait_for_message("/clock", Clock, timeout=5.0) 
        #         timer_1 = rospy.Time.now()
        #         time_spend_waiting = (timer_1 - timer_0).to_sec()
        #         print ("time_spend_waiting: ", time_spend_waiting)
        #     except:
        #         print "core dumped... kill"
        #         f_done = open(done_file,'w')
        #         f_done.write('1')
        #         f_done.close()
        #         break


          # if time_spend_waiting > 5.0:
          #   print "core dumped... kill"
          #   f_done = open(done_file,'w')
          #   f_done.write('1')
          #   f_done.close()

        """
        For this version, we consider 6 actions
        1-2) Increment/Decrement haa_joint
        3-4) Increment/Decrement hfe_joint
        5-6) Increment/Decrement kfe_joint
        """
        #self.action_space = spaces.Discrete(6)
        #self.reward_range = (-np.inf, np.inf)

        #self._seed()
        print ("end of init...")

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def _reset(self):
        # print("Pausing SIM...")
        # self.gazebo.pauseSim()
        # del self.mellingers
        # self.mellingers = [
        #     Mellinger_Agent(
        #         mav_name='hummingbird',
        #         index=0,
        #         num_agents=4,
        #         c=0.45,
        #         x=0.95, y=0.0, z=0.26,
        #         dim=3
        #     ),
        #     Mellinger_Agent(
        #         'hummingbird',
        #         index=1,
        #         num_agents=4,
        #         c=0.05,
        #         x=0.0, y=0.95, z=0.26,
        #         dim=3
        #     ),
        #     Mellinger_Agent(
        #         'hummingbird',
        #         index=2,
        #         num_agents=4,
        #         c=0.25,
        #         x=-0.95, y=0.0, z=0.26,
        #         dim=3
        #     ),
        #     Mellinger_Agent(
        #         'hummingbird',
        #         index=3,
        #         num_agents=4,
        #         c=0.25,
        #         x=0.0, y=-0.95, z=0.26,
        #         dim=3
        #     )
        # ]
        # print "pppppppppp"
        # 0st: We pause the Simulator


        # 1st: resets the simulation to initial values
        print("Reset SIM...")
        model_xml = rospy.get_param("robot_description")

        model_name = "four_quad_payload"
        rospy.wait_for_service('/gazebo/delete_model')
        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        del_model_prox(model_name)

        # pose = Pose()
        # pose.orientation.w = 1.0
        
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        # should this be unique so ros_control can use each individually?
        # req.robot_namespace = "/foo"
        # req.initial_pose = pose
        resp = spawn_model(req)
        time.sleep(2)
        self.gazebo.resetWorld()
        time.sleep(1)
        
        # self.gazebo.resetSim()
        
        # check_publishers_connection()
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
        # print("Remove Gravity...")
        # self.gazebo.change_gravity(0.0, 0.0, -9.81)
        
        # 7th: pauses simulation
        print("Pause SIM...")
        self.gazebo.pauseSim()
        time.sleep(0.2)

        # Get the State Discrete Stringuified version of the observations
        state = self.get_init_state()

        return state

    def _step(self, actions):
        
        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot

        # 1st, decide which action corresponsd to which joint is incremented

        # next_action_position = self.multi_uav_state_object.get_action_to_position(action)

        # update the reference in the mellinger controller from the action
        # for i, action in enumerate(actions):
        #     self.mellingers[i].set_ref_from_action(action)

        # We move it to that pos
        

        self.gazebo.unpauseSim()
        # action to the gazebo environment
        # hzyu
        # self.update_c_from_action(actions)

        reward_multi = [0,0,0,0]
        count = 0
        for i_mellinger in self.mellingers:
            # reward_multi[count] = -i_mellinger.reward_ft
            reward_multi[count] = -i_mellinger.reward_payload
            count=count+1
            now = time.time()
            i_mellinger.publish_poly3d_trj()

            i_mellinger.publish_err()

            i_mellinger.update_current_state()

            i_mellinger.update_des_distributor()

            i_mellinger.motorSpeedFromU()

            i_mellinger.send_motor_command()

        # self.rate.sleep()
        # hzyu
        # time.sleep(0.02)
        # self.monoped_joint_pubisher_object.move_joints(next_action_position)
        # Then we send the command to the robot and let it go
        # for running_step seconds
        self.gazebo.pauseSim()

        # We now process the latest data saved in the class state to calculate
        # the state and the rewards. This way we guarantee that they work
        # with the same exact data.
        # Generate State based on observations
        # observation = self.multi_uav_state_object.get_observations()

        # finally we get an evaluation based on what happened in the sim
        # reward,done = self.multi_uav_state_object.process_data()


        # Get the State Discrete Stringuified version of the observations
        # state = self.get_state(observation)
        state = self.get_step_state()

        return state, reward_multi

    def show_accumulated_force(self):
        for i_mellinger in self.mellingers:
            print "self.sum_force: ", i_mellinger.sum_force

    def compute_reward_arrive(self):
        self.mellingers[0].reward_arrive_goal(self.goal_position)
        print "reward arrive: ", self.mellingers[0].reward_arrive
        return self.mellingers[0].reward_arrive

    def set_goal_position_vel(self, goal_position, goal_velocity):
        for controller in self.mellingers:
            controller.NL_planner.setVerticesPosVel(goal_position, goal_velocity)

    def compute_trj(self, goal_position, goal_velocity):
        self.goal_position = np.array(goal_position)[-1, :].reshape((3, 1))
        print "ssssssssssssssss self.goal_position: ", self.goal_position
        self.set_goal_position_vel(goal_position, goal_velocity)
        for controller in self.mellingers:
            controller.optimize()
            controller.getPlanUpToSnap(frequency=50.0)

    def load_payload_references(self):
        for controller in self.mellingers:
            controller.load_ref_trj_payload()

    def hover_and_load_trj(self, dimension='xyz'):
        is_all_good = True
        count = 0
        while not rospy.is_shutdown():
            for i_mellinger in self.mellingers:
                if i_mellinger.hover_duration < 1.5:
                    is_all_good = False
                    break
            # print "distributor c: ", i_mellinger.c, "; distributor M0: ", i_mellinger                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       .get_M0()
            # print "distributor c_M0: ", i_mellinger.c_M0
            for i_mellinger in self.mellingers:
                #now = time.time()
                if i_mellinger.hover_duration < 1.5:
                    i_mellinger.set_hover_des(target_height=1.5)
                    
                if is_all_good:
                    i_mellinger.update_offset_xyz(i_mellinger.payload_position[0, 0], i_mellinger.payload_position[1, 0], i_mellinger.payload_position[2, 0])
                    i_mellinger.load_ref_trj_payload(dimension=dimension)
                    i_mellinger.offset_added = False
                    if not i_mellinger.offset_added:
                        print "hovering finished, going into the next phase..."
                        print "i_mellinger.positions_quads[2, 0]: ", i_mellinger.positions_quads[2, 0]

                        i_mellinger.update_offset_xyz(i_mellinger.positions_quads[0, 0], i_mellinger.positions_quads[1, 0], i_mellinger.positions_quads[2, 0])
                        i_mellinger.load_trj_lists(dimension=dimension)
                        # i_mellinger.update_offset()
                        # i_mellinger.load_trj_lists(dimension=dimension)
                        print "offset added: ", i_mellinger.name
                        count = count + 1
                    # i_mellinger.publish_poly3d_trj()
                    if count == 4:
                        # print "ssssssssssssss offset goal_position: ", self.goal_position
                        # print "ssssssssssssss self.mellingers[0].payload_position.reshape((3, 1)): ", self.mellingers[0].payload_position.reshape((3, 1))
                        self.goal_position = self.goal_position + self.mellingers[0].payload_position.reshape((3, 1))
                        # print "ssssssssssssss offset goal_position: ", self.goal_position

                        return True

                i_mellinger.publish_err()
                i_mellinger.update_current_state()
                i_mellinger.update_des_distributor()
                i_mellinger.motorSpeedFromU()
                i_mellinger.send_motor_command()
            self.rate.sleep()
            is_all_good = True

            

    def update_c_from_action(self, action):
        for i in range(self.num_agents):
            c = list(action[i*3:i*3+3])
            # print "c for ", i ,'th mellinger: ', c
            # self.mellingers[i].publish_poly3d_trj()
            self.mellingers[i].update_estimated_c(c)
            
            self.mellingers[i].update_current_state()
            # self.mellingers[i].update_des_distributor()

    def get_init_state(self):
        """
        We retrieve the Stringuified-Discrete version of the given observation
        :return: state
        """
        # return self.multi_uav_state_object.get_state_as_string(observation)

        return self.multi_uav_state_object.get_init_states()

    def get_step_state(self):
        """
        We retrieve the Stringuified-Discrete version of the given observation
        :return: state
        """
        # return self.multi_uav_state_object.get_state_as_string(observation)

        return self.multi_uav_state_object.get_step_states()
