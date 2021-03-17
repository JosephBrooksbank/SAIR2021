import numpy as np
import os

import random
import sys

import rospy

from Actions import Actions
from State import StateManager


class QTable:
    current_state = 0

    rewards = {}
    gamma = 0.8
    epsilon_modifier = 0
    epsilon = 0.2
    alpha = 0.2
    dir_path = os.path.dirname(os.path.realpath(__file__))
    txt_file = dir_path + "/q_table.csv"

    def __init__(self, state_manager):
        # type: (StateManager) -> None
        self.changed = False

        self.state_manager = state_manager
        self.NUM_STATES = self.state_manager.NUM_STATES
        self.actions = Actions()
        self.q_table = np.zeros((self.NUM_STATES, len(self.actions)))
        self.load_table()

        for val in range(0, self.NUM_STATES):
            self.rewards[val] = 0

        # setting negative rewards for being too close to any side
        for rightVal in self.state_manager.sensors[0].distances:
            for rightFrontVal in self.state_manager.sensors[1].distances:
                for frontVal in self.state_manager.sensors[2].distances:
                    for leftVal in self.state_manager.sensors[3].distances:
                        if rightVal == "too_far" \
                                or rightVal == "too_close" \
                                or frontVal == "too_close" \
                                or leftVal == "too_close":
                            self.rewards[self.state_manager \
                                .generate_state_index(rightVal, rightFrontVal, frontVal, leftVal)] = -1

    def update_state(self, state):
        # type: (int) -> None
        self.current_state = state
        self.changed = True

    def decrease_epsilon(self):
        self.epsilon_modifier += 1
        rospy.loginfo("New epsilon: " + str(self.diminishing_epsilon()))

    def diminishing_epsilon(self):
        return self.epsilon + (1.0/2)**self.epsilon_modifier * 0.7

    def next_action(self):
        state_for_action = self.current_state

        # Grabbing the column of actions available for the current state
        possible_actions = self.q_table[state_for_action, :]
        reward = -1000000.0
        # If no rewards present, go forward (for the first action)
        next_action = Actions.FORWARD
        # i is the index of the actions available to be taken at the current state
        if random.random() < self.diminishing_epsilon():
            for i in range(len(possible_actions)):
                if possible_actions[i] > reward:
                    next_action = i
                    reward = possible_actions[i]
        else:
            next_action = random.randint(0, len(possible_actions) - 1)
        self.actions.action_list[next_action]()
        new_state = self.wait_for_state()
        # rospy.loginfo(str(new_state) + " " + str(self.rewards[new_state]))
        # if self.rewards[new_state] == 0:
            # rospy.loginfo("In Goal State!")

        new_q = self.q_table[state_for_action, next_action] \
                + self.alpha * \
                (self.rewards[state_for_action] + self.gamma * \
                 self.calculate_max_action(new_state) - \
                 self.q_table[state_for_action, next_action])
        self.q_table[state_for_action, next_action] = new_q

    def wait_for_state(self):
        self.changed = False
        while not self.changed:
            pass
        return self.current_state

    def calculate_max_action(self, state):
        # type: (int) -> int
        max_reward = -1000000000000000.0
        possible_actions = self.q_table[state, :]
        for i in range(len(possible_actions)):
            if possible_actions[i] > max_reward:
                max_reward = possible_actions[i]

        return max_reward

    def save_table(self):
        np.savetxt(self.txt_file, self.q_table, fmt='%f', delimiter=",")

    def load_table(self):

        if os.path.exists(self.txt_file):
            self.q_table = np.loadtxt(self.txt_file, delimiter=",")
        else:
            # Q table is all zeros at the beginning
            self.q_table = np.zeros((self.NUM_STATES, len(self.actions)))
