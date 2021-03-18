import math
import sys


class State:
    all_distances = {
        "too_close": 0.4,
        "close": 0.60,
        "medium": 0.8,
        "far": 1.0,
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
        self.distances = ["close", "medium"]


class Front(State):
    def __init__(self):
        State.__init__(self)
        self.distances = ["too_close", "close", "medium", "far", "too_far"]


class Left(State):
    def __init__(self):
        State.__init__(self)
        self.distances = ["too_close", "too_far"]


class StateManager:
    sensors = [RightState(), RightFrontState(), Front(), Left()]

    NUM_STATES = int(math.pow(len(State.all_distances), len(sensors)))

    def __init__(self):
        pass

    def get_readings(self, *args):
        # type: (*float) -> None
        if len(args) != len(self.sensors):
            exit(-1)
        for i in range(len(args)):
            self.sensors[i].quantize_reading(args[i])

    def get_state_index(self):
        # type: () -> int
        # returns a unique index for every combination of the states
        # Generating index assuming all observation spaces have 5 discrete distance zones, because I couldn't think
        # of a way to do it otherwise. There will be empty holes in the Q table, but that is fine.
        index = 0
        for i in range(len(self.sensors)):
            temp = len(State.distances_index)**i * self.sensors[i].current_state_val()
            index += temp
        return index

    def generate_state_index(self, *args):
        # type: (*str) -> int
        if len(args) != len(self.sensors):
            exit(-1)
        index = 0
        for i in range(len(args)):
            index += len(State.distances_index)**i * State.distances_index[args[i]]

        return index
