#! /usr/bin/env python3

from enum import IntEnum


class Joy(IntEnum):
    DISCONNECTED = 1
    CONNECTED = 2
    NOT_WORKING = 3


class Motor(IntEnum):
    NOT_WORKING = 1
    FORWARD = 2
    BACKWARD = 3
    BREAKING = 4
    WAITING = 5


class State(IntEnum):
    DIRECTION_SEARCH = 0
    ROT_180 = 1
    ROT_90 = 2
    AVOID = 3
    NEXT = 4
    RIDE = 5
    CHECKING = 6
    FINISH = 7