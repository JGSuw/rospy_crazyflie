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
from rospy_crazyflie.motion_commands import *

class CrazyflieControl:

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
        print('foo')
        self._position_control_as = actionlib.SimpleActionServer(
            self._name + '/position_control',
            PositionControlAction,
            self._position_control_cb,
            False
        )
        print('bar')
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
        yaw_rate = req.yaw_rate
        self._cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z)
        return []

    def _set_param_cb(self, req):
        self._cf.param.set_value(req.param, req.value)
        print("set %s to %s" % (req.param, req.value))
        return SetParamResponse()

    def _velocity_control_cb(self, req):
        try:
            obj = pickle.loads(req.pickle)
            if isinstance(obj, SetVelSetpoint):
                self._mc.set_vel_setpoint(obj.vx, obj.vy, obj.vz, obj.rate_yaw)
            elif isinstance(obj, StartBack):
                self._mc.start_back(velocity = obj.velocity)
            elif isinstance(obj, StartCircleLeft):
                self._mc.start_circle_left(obj.radius_m, velocity = obj.velocity)
            elif isinstance(obj, StartCircleRight):
                self._mc.start_turn_right(obj.radius_m, velocity = obj.velocity)
            elif isinstance(obj, StartDown):
                self._mc.start_down(velocity = obj.velocity)
            elif isinstance(obj, StartForward):
                self._mc.start_forward(velocity = obj.velocity)
            elif isinstance(obj, StartLeft):
                self._mc.start_left(velocity = obj.velocity)
            elif isinstance(obj, StartLinearMotion):
                self._mc.start_linear_motion(obj.vx, obj.vy, obj.vz)
            elif isinstance(obj, StartRight):
                self._mc.start_right(velocity = obj.velocity)
            elif isinstance(obj, StartTurnLeft):
                self._mc.start_turn_left(rate = obj.rate)
            elif isinstance(obj, StartTurnRight):
                self._mc.start_turn_right(rate = obj.rate)
            elif isinstance(obj, StartUp):
                self._mc.start_up(velocity = obj.velocity)
            elif isinstance(obj, Stop):
                self._mc.stop()
            else:
                return 'Object is not a valid velocity command'
        except Exception as e:
            print(str(e))
            raise e
        return 'ok'


    """
    Action Implementations
    """

    def _position_control_cb(self, goal):
        try:
            obj = pickle.loads(goal.pickle)
            if isinstance(obj, Back):
                self._mc.back(obj.distance_m, velocity=obj.velocity)
            elif isinstance(obj, CircleLeft):
                self._mc.circle_left(obj.radius_m,
                    velocity = obj.velocity,
                    angle_degrees = obj.angle_degrees
                )
            elif isinstance(obj, CircleRight):
                self._mc.circle_right(obj.radius_m,
                    velocity = obj.velocity,
                    angle_degrees = obj.angle_degrees
                )
            elif isinstance(obj, Down):
                self._mc.down(obj.distance_m, velocity=obj.velocity)
            elif isinstance(obj, Forward):
                self._mc.forward(obj.distance_m, velocity=obj.velocity)
            elif isinstance(obj, Land):
                self._mc.land(velocity=obj.velocity)
            elif isinstance(obj, Left):
                self._mc.left(obj.distance_m, velocity=obj.velocity)
            elif isinstance(obj, MoveDistance):
                self._mc.move_distance(obj.x, obj.y, obj.z, velocity=obj.velocity)
            elif isinstance(obj, Right):
                self._mc.right(obj.distance_m, velocity=obj.velocity)
            elif isinstance(obj, TakeOff):
                self._mc.take_off(height=obj.height, velocity = obj.velocity)
            elif isinstance(obj, TurnLeft):
                self._mc.turn_left(obj.angle_degrees, rate=obj.rate)
            elif isinstance(obj, TurnRight):
                self._mc.turn_right(obj.angle_degrees, rate=obj.rate)
            elif isinstance(obj, Up):
                self._mc.up(obj.distance_m, velocity=obj.velocity)
        except Exception as e:
            print('Exception in action server %s' % self._name + '/position_control')
            print(str(e))
            print('Action aborted')
            self._position_control_as.set_aborted()
            return
        self._position_control_as.set_succeeded()

    def _takeoff(self, goal):
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
        self._move_distance_as.set_succeeded()
