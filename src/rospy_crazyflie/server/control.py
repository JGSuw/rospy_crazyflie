"""
Copyright (c) 2018, Joseph Sullivan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the <project name> project.
"""

import numpy as np
import pickle
import time

from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

import rospy
import actionlib

from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3

from rospy_crazyflie.msg import *
from rospy_crazyflie.srv import *
import rospy_crazyflie.motion_commands as motion_commands

vel_request_handlers = {
    'SetVelSetpoint':       lambda obj, mc: mc._set_vel_setpoint(obj['vx'], obj['vy'], obj['vz'], obj['rate_yaw']),
    'StartBack':            lambda obj, mc: mc.start_back(velocity = obj['velocity']),
    'StartCircleLeft':      lambda obj, mc: mc.start_circle_left(obj['radius_m'], velocity=obj['velocity']),
    'StartCircleRight':     lambda obj, mc: _mc.start_turn_right(obj['radius_m'], velocity=obj['velocity']),
    'StartDown':            lambda obj, mc: mc.start_down(velocity=obj['velocity']),
    'StartForward':         lambda obj, mc: mc.start_forward(velocity=obj['velocity']),
    'StartLeft':            lambda obj, mc: mc.start_left(velocity=obj['velocity']),
    'StartLinearMotion':    lambda obj, mc: mc_mc.start_linear_motion(obj['vx'], obj['vy'], obj['vz']),
    'StartRight':           lambda obj, mc: mc.start_right(velocity=obj['velocity']),
    'StartTurnLeft':        lambda obj, mc: mc.start_turn_left(rate=obj['rate']),
    'StartTurnRight':       lambda obj, mc:_mc.start_turn_right(rate=obj['rate']),
    'StartUp':              lambda obj, mc: mc.start_up(velocity=obj['velocity']),
    'Stop':                 lambda obj, mc: mc.stop(),
}

pos_request_handlers = {
    'Back':                 lambda obj, mc: mc.back(obj['distance_m'], velocity=obj['velocity']),
    'CircleLeft':           lambda obj, mc: mc.circle_left(obj['radius_m'], velocity=obj['velocity'], angle_degrees=obj['angle_degrees']),
    'CircleRight':          lambda obj, mc: mc.circle_right(obj['radius_m'], velocity=obj['velocity'], angle_degrees=obj['angle_degrees']),
    'Down':                 lambda obj, mc: mc.down(obj['distance_m'], velocity=obj['velocity']),
    'Forward':              lambda obj, mc: mc.forward(obj['distance_m'], velocity=obj['velocity']),
    'Land':                 lambda obj, mc: mc.land(velocity=obj['velocity']),
    'Left':                 lambda obj, mc: mc.left(obj['distance_m'], velocity=obj['velocity']),
    'MoveDistance':         lambda obj, mc: mc.move_distance(obj['x'], obj['y'], obj['z'], velocity=obj['velocity']),
    'Right':                lambda obj, mc: mc.right(obj['distance_m'], velocity=obj['velocity']),
    'TakeOff':              lambda obj, mc: mc.take_off(height=obj['height'], velocity=obj['velocity']),
    'TurnLeft':             lambda obj, mc: mc.turn_left(obj['angle_degrees'], rate=obj['rate']),
    'TurnRight':            lambda obj, mc: mc.turn_right(obj['angle_degrees'], rate=obj['rate']),
    'Up':                   lambda obj, mc: mc.up(obj['distance_m'], velocity=obj['velocity'])
}

class Control:

    def __init__(self, name, crazyflie):
        # Instantiate motion commander
        self._cf = crazyflie
        self._name = name
        self._mc = MotionCommander(self._cf)

        # Topic Publishers
        self._velocity_setpoint_pub = rospy.Publisher(
            self._name + '/velocity_setpoint',
            Vector3,
            queue_size = 10
        )

        """
        Services hosted for this crazyflie controller
        """
        self._reset_position_estimator_srv = rospy.Service(
            self._name + '/reset_position_estimator',
            ResetPositionEstimator,
            self._reset_position_estimator_cb
        )

        self._send_hover_setpoint_srv = rospy.Service(
            self._name + '/send_hover_setpoint',
            SendHoverSetpoint,
            self._send_hover_setpoint_cb
        )

        self._set_param_srv = rospy.Service(
            self._name + '/set_param',
            SetParam,
            self._set_param_cb
        )

        self._velocity_control_srv = rospy.Service(
            self._name + '/velocity_control',
            VelocityControl,
            self._velocity_control_cb
        )

        """
        Action servers for this crazyflie controller
        """
        self._position_control_as = actionlib.SimpleActionServer(
            self._name + '/position_control',
            PositionControlAction,
            self._position_control_cb,
            False
        )
        self._position_control_as.start()
    """
    Service Callbacks
    """
    def _reset_position_estimator_cb(self, req):
        pass

    def _send_hover_setpoint_cb(self, req):
        vx = req.vx
        vy = req.vy
        z = req.z
        z = req.z
        yaw_rate = req.yaw_rate
        self._cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z)
        return []

    def _set_param_cb(self, req):
        self._cf.param.set_value(req.param, req.value)
        print("set %s to %s" % (req.param, req.value))
        return SetParamResponse()

    def _velocity_control_cb(self, req):
        try:
            request = motion_commands.deserialize(req.request, self._mc)
            print(self._mc)
            vel_request_handlers[request['command']](reqeust)
        except Exception as e:
            print(str(e))
            raise e
        return 'ok'


    """
    Action Implementations
    """

    def _position_control_cb(self, goal):
        try:
            request = motion_commands.deserialize(goal.request)
            print(goal.request)
            pos_request_handlers[request['command']](request, self._mc)
        except Exception as e:
            print('Exception in action server %s' % self._name + '/position_control')
            print(str(e))
            print('Action aborted')
            self._position_control_as.set_aborted()
            return
        self._position_control_as.set_succeeded()

    def _takeoff(self, goal):
        # should this request be deprecated? appears to be handled by _position_control_cb
        try:
            self._mc.take_off(height = goal.height)
            time.sleep(5)
        except BaseException as e:
            self._takeoff_as.set_aborted()
            print(e)
            return
        self._takeoff_as.set_succeeded(TakeOffResult(True))

    def _land(self, goal):
        try:
            self._mc.land(velocity=goal.velocity)
        except BaseException as e:
            self._land_as.set_aborted()
            print(e)
            return
        self._land_as.set_succeeded(LandResult(True))

    def _move_distance(self, goal):
        # should this be deprecated? appears to be handled by _position_control_cb
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
            self._mc.move_distance(x, y, z, velocity = velocity)
            # self._velocity_setpoint_pub.publish(Vector3(vx, vy, vz))
        except BaseException as e:
            self._move_distance_as.set_aborted()
            print(e)
            return
