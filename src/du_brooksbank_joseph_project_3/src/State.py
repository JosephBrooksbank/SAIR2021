import math
import sys


class State:
    all_distances = {
        "too_close": 0.1,
        "close": 0.30,
        "medium": 0.6,
        "far": 0.8,
        "too_far": sys.maxint
    }
    distances_index = {
        "too_close": 0,
        "close": 1,
        "medium": 2,
        "far": 3,
        "too_far": 4
    }
    distances = []
    current_state = "too_far"

    def __init__(self):
        pass

    def __len__(self):
        # state object is basically a collection of which distances it can detect, so the length is the number of those
        return len(self.distances)

    def quantize_reading(self, distance_reading):
        for distance in self.distances:
            if distance_reading < self.all_distances[distance]:
                self.current_state = distance
                break

    def current_state_val(self):
        return self.distances_index[self.current_state]


class RightState(State):
    def __init__(self):
        State.__init__(self)
        self.distances = ["too_close", "close", "medium", "far", "too_far"]


class RightFrontState(State):
    def __init__(self):
        State.__init__(self)
        self.distances = ["far", "too_far"]


class Front(State):
    def __init__(self):
        State.__init__(self)
        self.distances = ["too_close", "close", "far", "too_far"]


class StateManager:
    right_state = RightState()
    right_front_state = RightFrontState()
    front_state = Front()
    NUM_STATES = int(math.pow(len(State.all_distances), 3))
    
    def __init__(self):
        pass

    def get_readings(self, r, rf, f):
        self.right_state.quantize_reading(r)
        self.right_front_state.quantize_reading(rf)
        self.front_state.quantize_reading(f)

    def get_state_index(self):
        # returns a unique index for every combination of the 3 states (max of 125 states, but only using 40)
        # Generating index assuming all 3 observation spaces have 5 discrete distance zones, because I couldn't think
        # of a way to do it otherwise. There will be empty holes in the Q table, but that is fine.
        return self.right_state.current_state_val() + 5 * self.right_front_state.current_state_val() \
               + 25 * self.front_state.current_state_val()

    def generate_state_index(self, r, rf, f):
        return self.right_state.distances.index(r) + 5 * self.right_front_state.distances.index(rf)\
                + 25 * self.front_state.distances.index(f)

    def index_valid(self, index):
        front_index = math.floor(index / 25)
        index = index % 25
        right_front_index = math.floor(index / 5)
        right_index = index % 5

        if (front_index >= len(self.front_state) or right_front_index >= len(self.right_front_state) \
                or right_index >= len(self.right_state)):
            return False
        return True
