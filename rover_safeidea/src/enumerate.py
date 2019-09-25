#! /usr/bin/env python3

from enum import IntEnum


class Joy(IntEnum):
    DISCONNECTED = 1
    CONNECTED = 2
    NOT_WORKING = 3
