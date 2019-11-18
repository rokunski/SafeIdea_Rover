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
    RIDE = 5
    CHECKING = 6
    FINISH = 7
    FIRST_AVOID = 13
    ROT_180_AVOID = 14
    ROT_AVOID = 15
    RIDE_AVOID = 16
    ROT_TO_PREV = 17
