from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig as cfLogConfig

import rospy
import actionlib
from rospy_crazyflie.msg import LogConfig, AddLogConfigAction, LogData
import json

class CrazyflieLog:
    """
    """

    def __init__(self, name, crazyflie):
        self._name = name
        # Create a crazyflie object
        self._cf = crazyflie
        # Set up the AddLogConfig Action server
        self._addLogConfig_as = actionlib.SimpleActionServer(
            name + '/add_log_config',
            AddLogConfigAction,
            self._addLogConfig_cb,
            False
        )
        self._addLogConfig_as.start()
        self._logs = {}

    def _addLogConfig_cb(self, goal):
        """ Implements the AddLogConfig Action"""
        # Construct the cflib LogConfig object
        config = goal.config
        new_config = cfLogConfig(config.name, config.period)
        new_config.data_received_cb.add_callback(self._log_data_cb)
        new_config.error_cb.add_callback(self._log_error_cb)
        for variable in config.variables:
            new_config.add_variable(variable.name, variable.type)
        self._logs[config.name] = new_config

        # Add this config to the Crazyflie
        try:
            self._cf.log.add_config(new_config)
            new_config.start()
            self._logs[config.name] = {}
            self._logs[config.name] = {'config' : new_config}
            self._logs[config.name]['publisher'] = rospy.Publisher(
                self._name + '/' + config.name,
                LogData,
                queue_size = 10
            )
            self._addLogConfig_as.set_succeeded()
            print("yippee")
        except BaseException as e:
            print("failed to add log %s" % (str(e)))
            self._addLogConfig_as.set_aborted()


    def _log_error_cb(self, logconf, msg):
        # TODO:
        pass

    def _log_data_cb(self, timestamp, data, logconf):
        new_data = LogData(json.dumps(data), timestamp)
        pub = self._logs[logconf.name]['publisher']
        pub.publish(new_data)

    def stop_logs(self):
        print("foo")
        for log_name in self._logs.keys():
            print("stopping log %s" %log_name)
            log = self._logs.pop(log_name)
            log['config'].stop()
            log['config'].delete()
            log['publisher'].unregister()
            del log
