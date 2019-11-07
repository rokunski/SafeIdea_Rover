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
