"""
JetArm Color Sorting Project
=============================

A PyBullet-based robotic simulation for sorting colored blocks using a 6-DOF robotic arm.
"""

__version__ = '2.0.0'
__author__ = 'JetArm Challenge Team'

from .main import *
from .jetarm_urdf_simulation import *

__all__ = ['JetArmSim', 'ColorBlockSortingChallenge']
