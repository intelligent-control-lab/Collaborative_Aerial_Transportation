#!/usr/bin/env python

'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Moded by Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com


'''
import gym
import time
import numpy
import random
import maddpg_training.common.tf_util as U
from maddpg_training.trainer.maddpg import MADDPGAgentTrainer
import tensorflow.contrib.layers as layers


from gym import wrappers
from std_msgs.msg import Float64

# ROS packages required
import rospy
import rospkg

# import our training environment
import multi_UAV_env

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

def lstm_model(lstm_input, lstm_state_in=None, num_outputs, scope, reuse=False, num_units=256, LSTM_UNITS=32, timestep_lstm=8, init=1):
    with tf.variable_scope(scope, reuse=reuse):
        out = lstm_input # (time_steps, batch_size, state_size)
        out = layers.fully_connected(out, num_outputs=num_units, activation_fn=tf.nn.relu)
        out = tf.reshape(out,[1, timestep_lstm, LSTM_UNITS])
        lstm_cell = tf.contrib.rnn.BasicLSTMCell(num_units=LSTM_UNITS, state_is_tuple=True)
        initial_lstm_state = lstm_cell.zero_state(1, tf.float32)
        if init == 1:
            lstm_outputs, lstm_state = tf.nn.dynamic_rnn(
                    lstm_cell,
                    out,
                    initial_state=lstm_state_in,
                    sequence_length=timestep_lstm,
                    time_major=False,
                    dtype=tf.float32
            )
        else if init ==0:
            lstm_outputs, lstm_state = tf.nn.dynamic_rnn(
                    lstm_cell,
                    out,
                    initial_state=lstm_state_in,
                    sequence_length=timestep_lstm,
                    time_major=False,
                    dtype=tf.float32
            )
        out = layers.fully_connected(lstm_outputs, num_outputs=num_outputs, activation_fn=None)
        return out,lstm_state






def mlp_model(mlp_input, num_outputs, scope, reuse=False, num_units=256, rnn_cell=None):
    # This model takes as input an observation and returns values of all actions
    with tf.variable_scope(scope, reuse=reuse):
        out = mlp_input
        out = layers.fully_connected(out, num_outputs=num_units, activation_fn=tf.nn.relu)
        out = layers.fully_connected(out, num_outputs=num_units, activation_fn=tf.nn.relu)
        out = layers.fully_connected(out, num_outputs=num_outputs, activation_fn=None)
        return out


def get_trainers(env,obs_shape_n, arglist):
    trainers = []
    critic_model = mlp_model
    policy_model = lstm_model
    trainer = MADDPGAgentTrainer
    for i in range(4):
        trainers.append(trainer(
            "agent_%d" % i, critic_model, policy_model, obs_shape_n, env.action_space, i, params,
            local_q_func=False))

    return trainers

if __name__ == '__main__':
    
    rospy.init_node('multi_UAV_gym', anonymous=True, log_level=rospy.INFO)





    # Create the Gym environment
    env = gym.make('multi_UAV-v0')
    rospy.logdebug ( "Gym environment done")


####
    reward_pub = rospy.Publisher('/multi_UAV/reward', Float64, queue_size=1)
    episode_reward_pub = rospy.Publisher('/multi_UAV/episode_reward', Float64, queue_size=1)

######


    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('collaborative_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.logdebug("Monitor Wrapper started")
    

    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file


    # Initialises the algorithm that we are going to use for learning


    start_time = time.time()
    





    with U.single_threaded_session():
        # Create agent trainers
        obs_shape_n = [env.observation_space[i].shape for i in range(4)]
        trainers = get_trainers(env, num_adversaries, obs_shape_n, arglist)

        # Initialize
        U.initialize()

        # Load previous results, if necessary
        if arglist.load_dir == "":
            arglist.load_dir = arglist.save_dir
        if arglist.display or arglist.restore or arglist.benchmark:
            print('Loading previous state...')
            U.load_state(arglist.load_dir)

        episode_rewards = [0.0]  # sum of rewards for all agents
        agent_rewards = [[0.0] for _ in range(env.n)]  # individual agent reward
        final_ep_rewards = []  # sum of rewards for training curve
        final_ep_ag_rewards = []  # agent rewards for training curve
        agent_info = [[[]]]  # placeholder for benchmarking info
        saver = tf.train.Saver()
        obs_n = env.reset()
        episode_step = 0
        train_step = 0
        t_start = time.time()

        print('Starting iterations...')
        while True:
            # get action
            action_n = [agent.action(obs) for agent, obs in zip(trainers,obs_n)]
            # environment step
            new_obs_n, rew_n, done_n, info_n = env.step(action_n)
            episode_step += 1
            done = all(done_n)
            terminal = (episode_step >= arglist.max_episode_len)
            # collect experience
            for i, agent in enumerate(trainers):
                agent.experience(obs_n[i], action_n[i], rew_n[i], new_obs_n[i], done_n[i], terminal)
            obs_n = new_obs_n

            for i, rew in enumerate(rew_n):
                episode_rewards[-1] += rew
                agent_rewards[i][-1] += rew

            if done or terminal:
                obs_n = env.reset()
                episode_step = 0
                episode_rewards.append(0)
                for a in agent_rewards:
                    a.append(0)
                agent_info.append([[]])

            # increment global step counter
            train_step += 1

            # for benchmarking learned policies
            if arglist.benchmark:
                for i, info in enumerate(info_n):
                    agent_info[-1][i].append(info_n['n'])
                if train_step > arglist.benchmark_iters and (done or terminal):
                    file_name = arglist.benchmark_dir + arglist.exp_name + '.pkl'
                    print('Finished benchmarking, now saving...')
                    with open(file_name, 'wb') as fp:
                        pickle.dump(agent_info[:-1], fp)
                    break
                continue

            # for displaying learned policies
            if arglist.display:
                time.sleep(0.1)
                env.render()
                continue

            # update all trainers, if not in display or benchmark mode
            loss = None
            for agent in trainers:
                agent.preupdate()
            for agent in trainers:
                loss = agent.update(trainers, train_step)

            # save model, display training output
            if terminal and (len(episode_rewards) % arglist.save_rate == 0):
                U.save_state(arglist.save_dir, saver=saver)
                # print statement depends on whether or not there are adversaries
                if num_adversaries == 0:
                    print("steps: {}, episodes: {}, mean episode reward: {}, time: {}".format(
                        train_step, len(episode_rewards), np.mean(episode_rewards[-arglist.save_rate:]), round(time.time()-t_start, 3)))
                else:
                    print("steps: {}, episodes: {}, mean episode reward: {}, agent episode reward: {}, time: {}".format(
                        train_step, len(episode_rewards), np.mean(episode_rewards[-arglist.save_rate:]),
                        [np.mean(rew[-arglist.save_rate:]) for rew in agent_rewards], round(time.time()-t_start, 3)))
                t_start = time.time()
                # Keep track of final episode reward
                final_ep_rewards.append(np.mean(episode_rewards[-arglist.save_rate:]))
                for rew in agent_rewards:
                    final_ep_ag_rewards.append(np.mean(rew[-arglist.save_rate:]))

            # saves final episode reward for plotting training curve later
            if len(episode_rewards) > arglist.num_episodes:
                rew_file_name = arglist.plots_dir + arglist.exp_name + '_rewards.pkl'
                with open(rew_file_name, 'wb') as fp:
                    pickle.dump(final_ep_rewards, fp)
                agrew_file_name = arglist.plots_dir + arglist.exp_name + '_agrewards.pkl'
                with open(agrew_file_name, 'wb') as fp:
                    pickle.dump(final_ep_ag_rewards, fp)
                print('...Finished total of {} episodes.'.format(len(episode_rewards)))
                break




    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.loginfo ("STARTING Episode #"+str(x))
        
        cumulated_reward = 0
        cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount
        
        # Initialize the environment and get first state of the robot
        rospy.logdebug("env.reset...")
        # Now We return directly the stringuified observations called state
        state = env.reset()

        rospy.logdebug("env.get_state...==>"+str(state))
        
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            
            # Execute the action in the environment and get feedback
            rospy.logdebug("###################### Start Step...["+str(i)+"]")
            rospy.logdebug("haa+,haa-,hfe+,hfe-,kfe+,kfe- >> [0,1,2,3,4,5]")
            rospy.logdebug("Action to Perform >> "+str(action))
            nextState, reward, done, info = env.step(action)
            rospy.logdebug("END Step...")
            rospy.logdebug("Reward ==> " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            rospy.logdebug("env.get_state...[distance_from_desired_point,base_roll,base_pitch,base_yaw,contact_force,joint_states_haa,joint_states_hfe,joint_states_kfe]==>" + str(nextState))

            # Make the algorithm learn based on the results
            qlearn.learn(state, action, reward, nextState)

            # We publish the cumulated reward
            cumulated_reward_msg.data = cumulated_reward
            reward_pub.publish(cumulated_reward_msg)

            if not(done):
                state = nextState
            else:
                rospy.logdebug ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

            rospy.logdebug("###################### END Step...["+str(i)+"]")

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = cumulated_reward
        episode_reward_pub.publish(episode_reward_msg)
        rospy.loginfo( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))

    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()
