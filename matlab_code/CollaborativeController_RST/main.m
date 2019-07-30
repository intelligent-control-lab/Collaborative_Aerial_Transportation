clear all
close all
clc

%%
syms x7 x8 x9 Ixx Iyy Izz Ixy Ixz Iyz;
skew_789 = [0 -x9 x8;
            x9 0 -x7;
            -x8 x7 0];
vector_789 = [x7; x8; x9];
J = [Ixx 0 0;
     0 Iyy 0;
     0 0 Izz];
J_inv = inv(J);
res = J_inv * (-skew_789 * (J*vector_789));

%% symboilc time derivatives
syms t phi theta psi p q r



%%
%The vechile parameters
maxRotorsVelocity = 2618; %[rad/s]
Omega_e = 6874; %[rad/s]

%The communication with the ROS server is closed if active
rosshutdown

%A new connection is estabilished with the ROS master
IP_ROS_Master = '10.161.48.181';
rosinit(IP_ROS_Master)
rostopic list

number_of_agent = 4;

%Topics will be used during the simulation
% 1st row: odometry ground truth
% 2nd row: force sensor data
sub_f_t = cell(2,number_of_agent);
sub_f_t{2,1} = rossubscriber('/hummingbird_0/f_t');
sub_f_t{2,2} = rossubscriber('/hummingbird_1/f_t');
sub_f_t{2,3} = rossubscriber('/hummingbird_2/f_t');
sub_f_t{2,4} = rossubscriber('/hummingbird_3/f_t');

sub_f_t{2,1} = rossubscriber('/hummingbird_0/ground_truth/odometry');
sub_f_t{2,2} = rossubscriber('/hummingbird_1/ground_truth/odometry');
sub_f_t{2,3} = rossubscriber('/hummingbird_2/ground_truth/odometry');
sub_f_t{2,4} = rossubscriber('/hummingbird_3/ground_truth/odometry');

[pub_0, msg_0] = rospublisher('/hummingbird_0/gazebo/command/motor_speed','mav_msgs/Actuators');
[pub_1, msg_1] = rospublisher('/hummingbird_1/gazebo/command/motor_speed','mav_msgs/Actuators');
[pub_2, msg_2] = rospublisher('/hummingbird_2/gazebo/command/motor_speed','mav_msgs/Actuators');
[pub_3, msg_3] = rospublisher('/hummingbird_3/gazebo/command/motor_speed','mav_msgs/Actuators');

msg.AngularVelocities = [100,100,100,100];
send(pub,msg)
