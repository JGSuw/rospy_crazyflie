
import rospy
import actionlib
import json
from rospy_crazyflie.msg import *
from rospy_crazyflie.srv import *


def log_decode(log_data):
    """ decodes log data from json string and returns python dict / list of the data
    """
    data = json.loads(log_data.data)
    timestamp = log_data.timestamp
    return (data, timestamp)

class CrazyflieClient:

    def __init__(self, name):
        self._name = name
        self._service_proxies = []
        self._action_clients = []

        self._takeoff_client = actionlib.SimpleActionClient(
            name + '/takeoff',
            TakeOffAction,
        )
        self._action_clients.append(self._takeoff_client)
        self._takeoff_client.wait_for_server()

        self._land_client = actionlib.SimpleActionClient(
            name + '/land',
            LandAction,
        )
        self._action_clients.append(self._land_client)
        self._land_client.wait_for_server()

        self._move_distance_client = actionlib.SimpleActionClient(
            name + '/move_distance',
            MoveDistanceAction,
        )
        self._action_clients.append(self._move_distance_client)
        self._move_distance_client.wait_for_server()

        self._add_logconfig_client = actionlib.SimpleActionClient(
            name + '/add_log_config',
            AddLogConfigAction,
        )
        self._action_clients.append(self._add_logconfig_client)
        self._add_logconfig_client.wait_for_server()

        # Service Clients
        self._set_param_client = rospy.ServiceProxy(
            name + '/set_param',
            SetParam
        )
        self._service_proxies.append(self._set_param_client)
        self._set_param_client.wait_for_service()

        self._send_hover_setpoint_client = rospy.ServiceProxy(
            name + '/send_hover_setpoint',
            SendHoverSetpoint
        )
        self._service_proxies.append(self._send_hover_setpoint_client)
        self._send_hover_setpoint_client.wait_for_service()

        self._disconnect_client = rospy.ServiceProxy(
            'crazyflie_server/disconnect',
            Disconnect
        )
        self._service_proxies.append(self._disconnect_client)
        self._disconnect_client.wait_for_service()

        self._send_hover_setpoint_client.wait_for_service()
        self._mc_actions = []
        self._current_mc_action = None
        self._log_subs = {}
        self._log_callbacks = {}

    # Methods
    """
    TODO:
    __delete__

    should stop all logs,
    abort all ongoing goals,
    unregister all subscriptions, service proxies, and action clients
    """

    def __delete__(self):
        self._land_client.send_goal(LandGoal(.25))
        self._land_client.wait_for_result()
        for action_client in self._action_clients:
            action_client.cancel_all_goals()
            del action_client
        self._disconnect_client.call(self._name)
        for service_proxy in self._service_proxies:
            service_proxy.close()
            del service_proxy
        for log_name in self._log_subs.keys():
            subscriber = self._log_subs.pop(log_name)
            subscriber.unregister()
            del subscriber


    def _do_mc_action(self):
        """ Executes the next motion control action in the queue
        """
        goal = self._current_mc_action[0]
        func_handle = self._current_mc_action[1]
        func_handle(goal, done_cb = self._motion_control_callback)

    def _add_mc_action(self, goal, func_handle):
        """ Adds a new motion control action to the queue

            @goal actionlib goal for this action
            @func_handle function which sends the goal to the corresponding actionserver
        """
        if self._current_mc_action is None:
            self._current_mc_action = (goal, func_handle)
            self._do_mc_action()
        else:
            self._mc_actions.append((goal, func_handle))

    def _motion_control_callback(self, state, result):
        """ This function is called when the current motion control action is
            no longer active.

            @state argument not used but provided by actionlib
            @result argument not used but provided by actionlib
        """
        if len(self._mc_actions) > 0:
            self._current_mc_action = self._mc_actions.pop(0)
            self._do_mc_action()
        else :
            self._current_mc_action = None

    def set_param(self, param, value):
        """ Changes the value of a parameter on this crazyflie
        """
        self._set_param_client(param, value)

    def takeoff(self, height):
        """ Causes the crazyflie to take off. This is a motion control action.
            It will go into a queue and be executed in turn with the other
            motion control actions.
        """
        goal = TakeOffGoal(height)
        self._add_mc_action(goal, self._takeoff_client.send_goal)

    def land(self, velocity):
        """ Causes the crazyflie to land. This is a motion control action.
            It will go into a queue and be executed in turn with the other
            motion control actions.
        """
        goal = LandGoal(velocity)
        self._add_mc_action(goal, self._land_client.send_goal)

    def move_distance(self, x,y,z,v):
        """ Causes the crazyflie to translate by the provided vector and at
            the specificed speed. This is a motion control action.
            It will go into a queue and be executed in turn with the other
            motion control actions.

            @x x displacemnet
            @y y displacmeent
            @z z displacement
            @v velocity
        """
        goal = MoveDistanceGoal(x,y,z,v)
        self._add_mc_action(goal, self._move_distance_client.send_goal)

    def send_hover_setpoint(self, vx, vy, yaw_rate, z):
        """ Sends a hover setpoint to the crazyflie. This specifies the vehicles
            x and y velocity, z position, and yaw rate.

            @vx x velocity
            @vy y velocity
            @z z position
            @yaw_rate yaw rate
        """
        self._send_hover_setpoint_client( vx, vy, yaw_rate, z)

    def add_log_config(self, name, variables, period_ms, callback = None):
        """ Adds a new log configuration to the crazyflie.

            @name name of the log configuration, it will be published with this name.
            @variables list of LogVariable objects which determine what data will be logged.
            @period_ms period, in millicseconds, between log downloads
            @callback optional callback, will be called with the log name and data
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
