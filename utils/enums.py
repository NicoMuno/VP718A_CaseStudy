from enum import Enum

class State(Enum):
    SLEEP = 0
    MANUAL = 1
    RECORD = 2
    REPLAY = 3

class Motion(Enum):
    STOP = 0
    STRAIGHT_FRONT= 1
    STRAIGHT_BACK= 2
    TURN_LEFT= 3
    TURN_RIGHT= 4
    