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
import rospy
import actionlib
import json
import pickle
import time

from rospy_crazyflie.msg import *
from rospy_crazyflie.srv import *
from ..motion_commands import *

def get_crazyflies(server='/crazyflie_server'):
        proxy = rospy.ServiceProxy(
            'crazyflie_server/get_crazyflies',
            GetCrazyflies
        )
        proxy.wait_for_service()
        response = proxy()
        return response.crazyflies

def log_decode(log_data):
    """ decodes log data from json string and returns python dict / list of the data
    """
    data = json.loads(log_data.data)
    timestamp = log_data.timestamp
    return (data, timestamp)

class Client:

    def __init__(self, name):
        self._name = name
        self._position_control_client = actionlib.SimpleActionClient(
            name + '/position_control',
            PositionControlAction
        )
        self._position_control_client.wait_for_server()

        self._add_logconfig_client = actionlib.SimpleActionClient(
            name + '/add_log_config',
            AddLogConfigAction,
        )
        self._add_logconfig_client.wait_for_server()

        # Service Clients

        self._velocity_control_client = rospy.ServiceProxy(
            name + '/velocity_control',
            VelocityControl
        )
        self._velocity_control_client.wait_for_service()

        self._set_param_client = rospy.ServiceProxy(
            name + '/set_param',
            SetParam
        )
        self._set_param_client.wait_for_service()

        self._send_hover_setpoint_client = rospy.ServiceProxy(
            name + '/send_hover_setpoint',
            SendHoverSetpoint
        )
        self._send_hover_setpoint_client.wait_for_service()

        self._mc_goals = []
        self._current_mc_goal = None
        self._log_subs = {}
        self._log_callbacks = {}

    # Methods
    """
    TODO: update comments to match bitcraze style
    """

    def __delete__(self):
        self.land()
        while self.action_in_progress():
            time.sleep(.1)
        self._position_control_client.cancel_all_goals()
        # for service_proxy in self._service_proxies:
        #     service_proxy.close()
        #     del service_proxy
        for log_name in self._log_subs.keys():
            subscriber = self._log_subs.pop(log_name)
            subscriber.unregister()
            del subscriber


    def _do_mc_action(self):
        """ Executes the next motion control action in the queue
        """
        goal = self._current_mc_goal
        self._position_control_client.send_goal(
            goal,
            done_cb = self._motion_control_callback
        )

    def _add_mc_goal(self, goal):
        """ Adds a new motion control action to the queue

            @goal actionlib goal for this action
            @func_handle function which sends the goal to the corresponding actionserver
        """
        if self._current_mc_goal is None:
            self._current_mc_goal = goal
            self._do_mc_action()
        else:
            self._mc_goals.append(goal)

    def _motion_control_callback(self, state, result):
        """
        This function is called when the current motion control action is
        no longer active.

        :param state: argument not used but provided by actionlib
        :param result: argument not used but provided by actionlib
        :return:
        """
        if len(self._mc_goals) > 0:
            self._current_mc_goal = self._mc_goals.pop(0)
            self._do_mc_action()
        else :
            self._current_mc_goal = None

    """
    Position Control Methods

    The interface is cloned directly from Bitcraze's craztflie-lib-python
    """

    def take_off(self, height=None, velocity=VELOCITY):
        """
        Takes off, that is starts the motors, goes straigt up and hovers.
        Do not call this function if you use the with keyword. Take off is
        done automatically when the context is created.

        :param height: the height (meters) to hover at. None uses the default
                       height set when constructed.
        :param velocity: the velocity (meters/second) when taking off
        :return:
        """
        action = TakeOff(height=height, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def land(self, velocity=VELOCITY):
        """
        Go straight down and turn off the motors.

        Do not call this function if you use the with keyword. Landing is
        done automatically when the context goes out of scope.

        :param velocity: The velocity (meters/second) when going down
        :return:
        """
        action = Land(velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def left(self, distance_m, velocity=VELOCITY):
        """
        Go left

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        action = Left(distance_m, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def right(self, distance_m, velocity=VELOCITY):
        """
        Go right

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        action = Right(distance_m, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def forward(self, distance_m, velocity=VELOCITY):
        """self._position_control
        Go forward

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        action = Forward(distance_m, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def back(self, distance_m, velocity=VELOCITY):
        """
        Go backwards

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        action = Back(distance_m, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def up(self, distance_m, velocity=VELOCITY):
        """
        Go up

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        action = Up(distance_m, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def down(self, distance_m, velocity=VELOCITY):
        """
        Go down

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        action = Down(distance_m, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def turn_left(self, angle_degrees, rate=RATE):
        """
        Turn to the left, staying on the spot

        :param angle_degrees: How far to turn (degrees)
        :param rate: The trurning speed (degrees/second)
        :return:
        """
        action = TurnLeft(angle_degrees, rate=rate)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)


    def turn_right(self, angle_degrees, rate=RATE):
        """
        Turn to the right, staying on the spot

        :param angle_degrees: How far to turn (degrees)
        :param rate: The trurning speed (degrees/second)
        :return:
        """
        action = TurnRight(angle_degrees, rate=rate)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def circle_left(self, radius_m, velocity=VELOCITY, angle_degrees=360.0):
        """
        Go in circle, counter clock wise

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity along the circle (meters/second)
        :param angle_degrees: How far to go in the circle (degrees)
        :return:
        """
        action = CircleLeft(radius_m, velocity=velocity, angle_degrees=angle_degrees)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def circle_right(self, radius_m, velocity=VELOCITY, angle_degrees=360.0):
        """
        Go in circle, clock wise

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity along the circle (meters/second)
        :param angle_degrees: How far to go in the circle (degrees)
        :return:
        """
        action = CircleRight(radius_m, velocity=velocity, angle_degrees=angle_degrees)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    def move_distance(self, distance_x_m, distance_y_m, distance_z_m,
                      velocity=VELOCITY):
        """
        Move in a straight line.
        positive X is forward
        positive Y is left
        positive Z is up

        :param distance_x_m: The distance to travel along the X-axis (meters)
        :param distance_y_m: The distance to travel along the Y-axis (meters)
        :param distance_z_m: The distance to travel along the Z-axis (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        action = MoveDistance(distance_x_m, distance_y_m, distance_y_m, velocity=velocity)
        goal = PositionControlGoal(action.serialize())
        self._add_mc_goal(goal)

    """
    Velocity Control Methods
    The interface is cloned directly from Bitcraze's craztflie-lib-python
    """
    def start_left(self, velocity=VELOCITY):
        """
        Start moving left. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartLeft(velocity=velocity)
        self._velocity_control_client(action.serialize())

    def start_right(self, velocity=VELOCITY):
        """
        Start moving right. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartRight(velocity=velocity)
        self._velocity_control_client(action.serialize())

    def start_forward(self, velocity=VELOCITY):
        """
        Start moving forward. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartForward(velocity=velocity)
        self._velocity_control_client(action.serialize())

    def start_back(self, velocity=VELOCITY):
        """
        Start moving backwards. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartBack(velocity=velocity)
        self._velocity_control_client(action.serialize())

    def start_up(self, velocity=VELOCITY):
        """
        Start moving up. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartUp(velocity=velocity)
        self._velocity_control_client(action.serialize())

    def start_down(self, velocity=VELOCITY):
        """
        Start moving down. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartDown(velocity=velocity)
        self._velocity_control_client(action.serialize())

    def stop(self):
        """
        Stop any motion and hover.

        :return:
        """
        self._current_mc_goal = None
        self._mc_goals = []
        self._position_control_client.cancel_all_goals()
        action = Stop()
        self._velocity_control_client(action.serialize())

    def start_turn_left(self, rate=RATE):
        """
        Start turning left. This function returns immediately.

        :param rate: The angular rate (degrees/second)
        :return:
        """
        action = StartTurnLeft(rate=rate)
        self._velocity_control_client(action.serialize())

    def start_turn_right(self, rate=RATE):
        """
        Start turning right. This function returns immediately.

        :param rate: The angular rate (degrees/second)
        :return:
        """
        action = StartTurnRight(rate=rate)
        self._velocity_control_client(action.serialize())

    def start_circle_left(self, radius_m, velocity=VELOCITY):
        """
        Start a circular motion to the left. This function returns immediately.

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartCircleLeft(radius_m, velocity=velocity)
        self._velocity_control_client(action.serialize())

        # self._set_vel_setpoint(velocity, 0.0, 0.0, -rate) WHY WAS THIS HERE?

    def start_circle_right(self, radius_m, velocity=VELOCITY):
        """
        Start a circular motion to the right. This function returns immediately

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        action = StartCircleRight(radius_m, velocity=velocity)
        self._velocity_control_client(action.serialize())

    def start_linear_motion(self, velocity_x_m, velocity_y_m, velocity_z_m):
        """
        Start a linear motion. This function returns immediately.

        positive X is forward
        positive Y is left
        positive Z is up

        :param velocity_x_m: The velocity along the X-axis (meters/second)
        :param velocity_y_m: The velocity along the Y-axis (meters/second)
        :param velocity_z_m: The velocity along the Z-axis (meters/second)
        :return:
        """
        action = StartLinearMotion(velocity_x_m, velocity_y_m, velocity_z_m)
        self._velocity_control_client(action.serialize())

    def _set_vel_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        action = SetVelSetpoint(velocity_x, velocity_y, velocity_z, rate_yaw)
        self._velocity_control_client(action.serialize())

    def set_param(self, param, value):
        """
        Changes the value of a parameter on this crazyflie
        """
        self._set_param_client(param, value)

    def send_hover_setpoint(self, vx, vy, yaw_rate, z):
        """
        Sends a hover setpoint to the crazyflie. This specifies the vehicles
        x and y velocity, z position, and yaw rate.

        :param vx: x velocity
        :param vy: y velocity
        :param z: position
        :param yaw_rate: yaw rate
        :return:
        """
        self._send_hover_setpoint_client( vx, vy, yaw_rate, z)

    def add_log_config(self, name, variables, period_ms, callback = None):
        """
        Adds a new log configuration to the crazyflie.

        :param name: name of the log configuration, it will be published with this name.
        :param variables: list of LogVariable objects which determine what data will be logged.
        :param period_ms: in millicseconds, between log downloads
        :param callback: optional callback, will be called with the log name and data
        :return:
        """

        log_config = LogConfig(name, variables, period_ms)
        self._add_logconfig_client.send_goal(
            AddLogConfigGoal(log_config)
        )
        self._add_logconfig_client.wait_for_result()
        result = self._add_logconfig_client.get_state()

        print("result of add log config for log %s is %d" % (name, result))

        if result == actionlib.GoalStatus.SUCCEEDED:
            self._log_subs[log_config.name] = rospy.Subscriber (
                self._name + '/' + log_config.name,
                LogData,
                callback = self._log_data_cb,
                callback_args = log_config.name
            )
            self._log_callbacks[log_config.name] = callback
        else:
            pass

    def _log_data_cb(self, log_data, log_name):
        (data, timestamp) = log_decode(log_data)
        if self._log_callbacks[log_name] is not None:
            self._log_callbacks[log_name](data, timestamp)

    def _actions_in_waiting(self):
        return len(self._mc_goals)

    def action_in_progress(self):
        if self._current_mc_goal is not None:
            return True
        elif self._actions_in_waiting() > 0:
            return True
        return None

    def wait(self):
        while self.action_in_progress():
            time.sleep(.1)
