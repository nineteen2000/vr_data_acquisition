"""LHandPro grasp presets for six motor position commands."""

from turtle import heading

from launch_ros.events.lifecycle import state_transition
from numpy import info


HAND_DOF = 6

PRESETS = {
    "open": [0, 0, 0, 0, 0, 0],
    "middle": [3000, 3000, 3000, 3000, 3000, 3000],
    "close": [6000, 6000, 6000, 6000, 6000, 6000],
}





