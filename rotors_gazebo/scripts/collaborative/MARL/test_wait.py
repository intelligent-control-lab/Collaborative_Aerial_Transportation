import rospy
from sensor_msgs.msg import JointState
import time

done_file  = '/home/lucasyu/catkin_ws/src/collaborative_transportation/rotors_gazebo/scripts/collaborative/MARL/maddpg_training/done.txt'

rosout_old = None
def callback_rousout(msg):
	# print msg.header
	global rosout_old
	rosout_old = msg.header


if __name__ == '__main__':
	rospy.init_node('test_rosout', anonymous=False, log_level=rospy.INFO)
	cb = rospy.Subscriber('/joint_states', JointState, callback_rousout)
	k = 0
	while True:
		if rosout_old == None:
	        print "core dumped... kill"
	        f_done = open(done_file,'w')
	        f_done.write('1')
	        f_done.close()
	    else:
	    	rosout_old = None
		time.sleep(0.01)
