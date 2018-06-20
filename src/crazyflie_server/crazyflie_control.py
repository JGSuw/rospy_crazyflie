import numpy as np
import time

from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

import rospy
import actionlib

from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3

# import actionlib messages
from rospy_crazyflie.msg import TakeOffAction, TakeOffGoal, TakeOffResult
from rospy_crazyflie.msg import LandAction, LandGoal, LandResult
from rospy_crazyflie.msg import MoveDistanceAction, MoveDistanceGoal

# Service definitions
from rospy_crazyflie.srv import SetParam, SetParamResponse
from rospy_crazyflie.srv import SendHoverSetpoint

class CrazyflieControl:

    def __init__(self, name, crazyflie):
        # Instantiate motion commander
        self._cf = crazyflie
        self._name = name
        self.mc = MotionCommander(self._cf)

        # Topic Publishers
        self._velocity_setpoint_pub = rospy.Publisher(
            self._name + '/velocity_setpoint',
            Vector3,
            queue_size = 10
        )

        # Service servers
        self._set_param_srv = rospy.Service(
            self._name + '/set_param',
            SetParam,
            self._set_param_cb
        )

        self._send_hover_setpoint_srv = rospy.Service(
            self._name + '/send_hover_setpoint',
            SendHoverSetpoint,
            self._send_hover_setpoint_cb
        )

        # Set up ros actions
        self._takeoff_as = actionlib.SimpleActionServer(
            self._name + '/takeoff',
            TakeOffAction,
            self._takeoff,
            False
        )
        self._takeoff_as.start()

        self._land_as = actionlib.SimpleActionServer(
            self._name + '/land',
            LandAction,
            self._land,
            False
        )
        self._land_as.start()

        self._move_distance_as = actionlib.SimpleActionServer(
            self._name + '/move_distance',
            MoveDistanceAction,
            self._move_distance,
            False
        )
        self._move_distance_as.start()

    def _set_param_cb(self, req):
        self._cf.param.set_value(req.param, req.value)
        print("set %s to %s" % (req.param, req.value))
        return SetParamResponse()

    def _send_hover_setpoint_cb(self, req):
        vx = req.vx
        vy = req.vy
        z = req.z
        yaw_rate = req.yaw_rate
        self._cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z)
        return []

    def _takeoff(self, goal):
        try:
            self.mc.take_off(height = goal.height)
            time.sleep(5)
        except BaseException as e:
            self._takeoff_as.set_aborted()
            print(e)
            return
        self._takeoff_as.set_succeeded(TakeOffResult(True))

    def _land(self, goal):
        try:
            self.mc.land(velocity=goal.velocity)
        except BaseException as e:
            self._land_as.set_aborted()
            print(e)
            return
        self._land_as.set_succeeded(LandResult(True))

    def _move_distance(self, goal):
        try:
            x = goal.x
            y = goal.y
            z = goal.z
            velocity = goal.velocity

            dist = np.sqrt(x**2 + y**2 + z**2)
            vx = x / dist * velocity
            vy = y / dist * velocity
            vz = z / dist * velocity

            # self._velocity_setpoint_pub.publish(Vector3(vx, vy, vz))
            self.mc.move_distance(x, y, z, velocity = velocity)
            # self._velocity_setpoint_pub.publish(Vector3(vx, vy, vz))
        except BaseException as e:
            self._move_distance_as.set_aborted()
            print(e)
            return
        self._move_distance_as.set_succeeded()
