#!/usr/bin/env python

import gym
import numpy
import time
import qlearn
from gym import wrappers
import rospy
import rospkg
# import our training environment
from openai_ros.task_envs.wamv import wamv_nav_twosets_buoys


if __name__ == '__main__':

    rospy.init_node('barco_qlearn', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('WamvNavTwoSetsBuoys-v0')
    rospy.loginfo("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('barco_openai')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/wamv/alpha")
    Epsilon = rospy.get_param("/wamv/epsilon")
    Gamma = rospy.get_param("/wamv/gamma")
    epsilon_discount = rospy.get_param("/wamv/epsilon_discount")
    nepisodes = rospy.get_param("/wamv/nepisodes")
    nsteps = rospy.get_param("/wamv/nsteps")

    # Initialize Q-Learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n), alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    # Episodes
    for x in range(nepisodes):
        rospy.logdebug("### START EPISODE => " + str(x))

        # Reward cumulated in each episode
        cumulated_reward = 0
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon = qlearn.epsilon - (qlearn.epsilon * epsilon_discount)

        # Reset the evnironment and get first state
        observation = env.reset()
        state = ''.join(map(str, observation))

        # Steps for training in episode
        for i in range(nsteps):

            rospy.logwarn("### START Step => " + str(i))

            # Choose action
            action = qlearn.chooseAction(state)

            rospy.logwarn("Next action: %d", action)

            # Execute the action and get reward
            observation, reward, done, info = env.step(action)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            rospy.logwarn("Reward: " + str(reward))
            rospy.logwarn("Episode cumulated_reward: " + str(cumulated_reward))

            # Get next state where the robot arrived
            nextState = ''.join(map(str, observation))

            # Update Q-values
            qlearn.learn(state, action, reward, nextState)

            if not (done):
                rospy.logwarn("NOT DONE")
                state = nextState
            else:
                rospy.logwarn("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

            rospy.logwarn("### END Step => " + str(i))

        # Finished episode
        minutos, segundos = divmod(int(time.time() - start_time), 60)
        horas, minutos = divmod(minutos, 60)

        rospy.logerr(("EP: " + str(x + 1) + " - [Alpha: " + str(round(qlearn.alpha, 2)) + " - Gamma: " + str(round(qlearn.gamma, 2)) + 
            " - Epsilon: " + str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(cumulated_reward) + 
            "     Time: %d:%02d:%02d" % (horas, minutos, segundos)))

    # Finished training
    minutos, segundos = divmod(int(time.time() - start_time), 60)
    horas, minutos = divmod(m, 60)

    rospy.logerr(("Episodes: " + str(x + 1) + " - [Epsilon: " + str(round(qlearn.epsilon, 2)) + " | Highest reward: " + str(highest_reward) + 
        "] Time: %d:%02d:%02d" % (horas, minutos, segundos)))


    rospy.loginfo(("\n|" + str(nepisodes) + "|" + str(qlearn.alpha) + "|" + str(qlearn.gamma) + "|" + str(
        initial_epsilon) + "*" + str(epsilon_discount) + "|" + str(highest_reward) + "| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()