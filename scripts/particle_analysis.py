#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_collision_detection')
import rospy
from robot_collision_detection.msg import CollPart
from robot_collision_detection.srv import GetPart

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TwistStamped, WrenchStamped
from qb_interface.msg import handRef, handPos
from std_msgs.msg import UInt32MultiArray, String
import time
import numpy
