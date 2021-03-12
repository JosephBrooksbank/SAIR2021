import numpy as np
import os

import random

import rospy

from Actions import Actions
from State import State, StateManager


class QTable:

    # Current state is updated in a callback in the main class (gross) # TODO Can I change this
    current_state = -1

    rewards = {}
    gamma = 0.8
    epsilon = 0.9
    dir_path = os.path.dirname(os.path.realpath(__file__))
    txt_file = dir_path + "/q_table.csv"

    # TODO find a better way to get this value in this class
    NUM_STATES = 125

    def __init__(self):
        self.state_manager = StateManager()
        self.actions = Actions()
        self.q_table = np.zeros((self.NUM_STATES, len(self.actions)))
        self.load_table()

        for val in range(0, self.NUM_STATES):
            self.rewards[val] = 0
        # Setting rewards for being a medium distance away from the wall, and not too close to the front
        self.rewards[self.state_manager.generate_state_index("medium", "far", "far")] = 100
        self.rewards[self.state_manager.generate_state_index("medium", "far", "too_far")] = 100
        # setting negative rewards for being too close to any side
        for rightVal in self.state_manager.right_state.distances:
            for rightFrontVal in self.state_manager.right_front_state.distances:
                for frontVal in self.state_manager.front_state.distances:
                    if rightVal == "too_close" or rightVal == "too_far" or frontVal == "too_close":
                        self.rewards[self.state_manager.generate_state_index(rightVal, rightFrontVal, frontVal)] = -1

    def next_action(self):
        state_for_action = self.current_state
        # Grabbing the column of actions available for the current state
        possible_actions = self.q_table[state_for_action, :]
        reward = -100
        # If no rewards present, go forward (for the first action)
        next_action = Actions.FORWARD
        # i is the index of the actions available to be taken at the current state
        if random.random() < self.epsilon:
            for i in range(len(possible_actions)):
                if possible_actions[i] > reward:
                    next_action = i
                    reward = possible_actions[i]
        else:
            rospy.loginfo("Took random action!")
            next_action = random.randint(0, len(possible_actions)-1)
        self.actions.action_list[next_action]()
        new_state = self.current_state
        if self.rewards[new_state] == 100:
            rospy.loginfo("In Goal State!")
        self.q_table[state_for_action, next_action] = self.rewards[new_state] + self.calculate_reward(new_state)

    def calculate_reward(self, state):
        max_reward = -1
        possible_actions = self.q_table[state, :]
        for i in range(len(possible_actions)):
            if possible_actions[i] > max_reward:
                max_reward = possible_actions[i]

        return self.gamma * max_reward

    def save_table(self):
        np.savetxt(self.txt_file, self.q_table, delimiter=",")

    def load_table(self):

        if os.path.exists(self.txt_file):
            self.q_table = np.loadtxt(self.txt_file, delimiter=",")
        else:
            # Q table is all zeros at the beginning
            self.q_table = np.zeros((self.NUM_STATES, len(self.actions)))


