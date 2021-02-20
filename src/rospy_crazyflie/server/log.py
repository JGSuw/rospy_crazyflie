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

from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig as cfLogConfig

import rospy
import actionlib
from rospy_crazyflie.msg import *
import json

class Log:
    """
    This object manages the creation of vehicle telemetry logging configurations,
    and publishing log data to ros topics.
    """

    def __init__(self, name, crazyflie):
        self._name = name
        self._cf = crazyflie

        # Set up the AddLogConfig Action server for custom / ad hoc log configurations
        self._addLogConfig_as = actionlib.SimpleActionServer(
            name + '/add_log_config',
            AddLogConfigAction,
            self._addLogConfig_cb,
            False
        )
        self._addLogConfig_as.start()
        self._logs = {}

        # Set up log configs for standard log groups
        self._controller_rpy_rate_pub = rospy.Publisher(
            self._name + '/ControllerRPYRate',
            ControllerRPYRate,
            queue_size=100
        )
        self._controller_rpy_rate_config = cfLogConfig(name='ControllerRPYRate', period_in_ms=100)
        self._controller_rpy_rate_config.data_received_cb.add_callback(self._controller_rpy_rate_cb)
        self._controller_rpy_rate_config.add_variable('controller.rollRate', 'float')
        self._controller_rpy_rate_config.add_variable('controller.pitchRate', 'float')
        self._controller_rpy_rate_config.add_variable('controller.yawRate', 'float')
        self._cf.log.add_config(self._controller_rpy_rate_config)

        self._controller_rpyt_pub = rospy.Publisher(
            self._name + '/ControllerRPYT',
            ControllerRPYT,
            queue_size=100
        )
        self._controller_rpyt_config = cfLogConfig(name='ControllerRPYT', period_in_ms=100)
        self._controller_rpyt_config.data_received_cb.add_callback(self._controller_rpyt_cb)
        self._controller_rpyt_config.add_variable('controller.actuatorThrust', 'float')
        self._controller_rpyt_config.add_variable('controller.roll', 'float')
        self._controller_rpyt_config.add_variable('controller.pitch', 'float')
        self._controller_rpyt_config.add_variable('controller.yaw', 'float')
        self._cf.log.add_config(self._controller_rpyt_config)

        self._kalman_position_pub = rospy.Publisher(
            self._name + '/KalmanPositionEst',
            KalmanPositionEst,
            queue_size=100
        )
        self._kalman_position_config = cfLogConfig(name='KalmanPositionEst', period_in_ms=100)
        self._kalman_position_config.data_received_cb.add_callback(self._kalman_position_cb)
        self._kalman_position_config.add_variable('kalman.stateX', 'float')
        self._kalman_position_config.add_variable('kalman.stateY', 'float')
        self._kalman_position_config.add_variable('kalman.stateZ', 'float')
        self._cf.log.add_config(self._kalman_position_config)

        self._motor_power_pub = rospy.Publisher(
            self._name + '/MotorPower',
            MotorPower,
            queue_size=100
        )
        self._motor_power_config = cfLogConfig(name='MotorPower', period_in_ms=100)
        self._motor_power_config.data_received_cb.add_callback(self._motor_power_cb)
        self._motor_power_config.add_variable('motor.m4', 'int32_t')
        self._motor_power_config.add_variable('motor.m1', 'int32_t')
        self._motor_power_config.add_variable('motor.m2', 'int32_t')
        self._motor_power_config.add_variable('motor.m3', 'int32_t')
        self._cf.log.add_config(self._motor_power_config)

        self._posCtl_pub = rospy.Publisher(
            self._name + '/posCtl',
            posCtl,
            queue_size=100
        )
        self._posCtl_config = cfLogConfig(name='posCtl', period_in_ms=100)
        self._posCtl_config.data_received_cb.add_callback(self._pos_ctl_cb)
        self._posCtl_config.add_variable('posCtl.targetVX', 'float')
        self._posCtl_config.add_variable('posCtl.targetVY', 'float')
        self._posCtl_config.add_variable('posCtl.targetVZ', 'float')
        self._posCtl_config.add_variable('posCtl.targetX', 'float')
        self._posCtl_config.add_variable('posCtl.targetY', 'float')
        self._posCtl_config.add_variable('posCtl.targetZ', 'float')
        self._cf.log.add_config(self._posCtl_config)

        self._stabilizer_pub = rospy.Publisher(
            self._name + '/Stabilizer',
            Stabilizer,
            queue_size=100
        )
        self._stabilizer_config = cfLogConfig(name='Stabilizer', period_in_ms=100)
        self._stabilizer_config.data_received_cb.add_callback(self._stabilizer_cb)
        self._stabilizer_config.add_variable('stabilizer.roll', 'float')
        self._stabilizer_config.add_variable('stabilizer.pitch', 'float')
        self._stabilizer_config.add_variable('stabilizer.yaw', 'float')
        self._stabilizer_config.add_variable('stabilizer.thrust', 'uint16_t')
        self._cf.log.add_config(self._stabilizer_config)

    def _addLogConfig_cb(self, goal):
        """ Implements the AddLogConfig Action"""
        # Construct the cflib LogConfig object
        config = goal.config
        new_config = cfLogConfig(name=config.name, period_in_ms=config.period)
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
                queue_size = 100
            )
            self._addLogConfig_as.set_succeeded()
        except BaseException as e:
            print("failed to add log %s" % (str(e)))
            self._addLogConfig_as.set_aborted()


    def _log_error_cb(self, logconf, msg):
        """
        Error handler from CrazyflieLibPython for all custom / ad hoc log configurations
        """
        # TODO:
        pass

    def _log_data_cb(self, timestamp, data, logconf):
        """
        Callback from CrazyflieLibPython for all custom / ad hoc log configurations
        """
        new_data = LogData(json.dumps(data), timestamp)
        pub = self._logs[logconf.name]['publisher']
        pub.publish(new_data)

    def stop_logs(self):
        """
        Stops log configurations,
        """
        for log_name in self._logs.keys():
            print("stopping log %s" %log_name)
            log = self._logs.pop(log_name)
            log['config'].stop()
            log['config'].delete()
            log['publisher'].unregister()
            del log

        self.stop_log_controller_rpy_rate()
        self.stop_log_controller_rpyt()
        self.stop_log_kalman_position_est()
        self.stop_log_motor_power()
        self.stop_log_pos_ctl()
        self.stop_log_stabilizer()

    def log_controller_rpy_rate(self, period_in_ms=100):
        """ Starts ControllerRPYRate log config"""
        self._controller_rpy_rate_config.period_in_ms = period_in_ms
        self._controller_rpy_rate_config.start()

    def log_controller_rpyt(self, period_in_ms=100):
        """ Starts ControllerRPYT log config """
        self._controller_rpyt_config.period_in_ms = period_in_ms
        self._controller_rpyt_config.start()

    def log_kalman_position_est(self, period_in_ms=100):
        """ Starts KalmanPositionEst log config """
        self._kalman_position_config.period_in_ms = period_in_ms
        self._kalman_position_config.start()

    def log_motor_power(self, period_in_ms=100):
        """ Starts MotorPower log config """
        self._motor_power_config.period_in_ms = period_in_ms
        self._motor_power_config.start()

    def log_pos_ctl(self, period_in_ms=100):
        """ Starts MotorPower log config """
        self._posCtl_config.period_in_ms = period_in_ms
        self._posCtl_config.start()

    def log_stabilizer(self, period_in_ms=100):
        """ Starts Stabilizer log config """
        self._stabilizer_config.period_in_ms = period_in_ms
        self._stabilizer_config.start()

    def stop_log_controller_rpy_rate(self):
        """ Stops ControllerRPYRate log config"""
        self._controller_rpy_rate_config.stop()

    def stop_log_controller_rpyt(self):
        """ Stops ControllerRPYT log config """
        self._controller_rpyt_config.stop()

    def stop_log_kalman_position_est(self):
        """ Stops KalmanPositionEst log config """
        self._kalman_position_config.stop()

    def stop_log_motor_power(self):
        """ Stops MotorPower log config """
        self._motor_power_config.stop()

    def stop_log_pos_ctl(self):
        """ Stops MotorPower log config """
        self._posCtl_config.stop()

    def stop_log_stabilizer(self):
        """ Stops Stabilizer log config """
        self._stabilizer_config.stop()

    def _controller_rpy_rate_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes ControllerRPYRate messages """
        msg = ControllerRPYRate()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.rollRate = data['controller.rollRate']
        msg.pitchRate = data['controller.pitchRate']
        msg.yawRate = data['controller.yawRate']
        self._controller_rpy_rate_pub.publish(msg)

    def _controller_rpyt_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes ControllerRPYT messages """
        msg = ControllerRPYT()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.roll = data['controller.roll']
        msg.pitch = data['controller.pitch']
        msg.yaw = data['controller.yaw']
        msg.actuatorThrust = data['controller.actuatorThrust']
        self._controller_rpyt_pub.publish(msg)

    def _kalman_position_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes KalmanPositionEst messages """
        msg = KalmanPositionEst()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.stateX = data['kalman.stateX']
        msg.stateY = data['kalman.stateY']
        msg.stateZ = data['kalman.stateZ']
        self._kalman_position_pub.publish(msg)

    def _motor_power_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes MotorPower messages """
        msg = MotorPower()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.m4 = data['motor.m4']
        msg.m1 = data['motor.m1']
        msg.m2 = data['motor.m2']
        msg.m3 = data['motor.m3']
        self._motor_power_pub.publish(msg)

    def _pos_ctl_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes posCtl messages """
        msg = posCtl()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.targetVX = data['posCtl.targetVX']
        msg.targetVY = data['posCtl.targetVY']
        msg.targetVZ = data['posCtl.targetVZ']
        msg.targetX = data['posCtl.targetX']
        msg.targetY = data['posCtl.targetY']
        msg.targetZ = data['posCtl.targetZ']
        self._posCtl_pub.publish(msg)

    def _stabilizer_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Stabilizer messages """
        msg = Stabilizer()
        msg.stamp = rospy.get_rostime()
        msg.cfstamp = timestamp
        msg.roll = data['stabilizer.roll']
        msg.pitch = data['stabilizer.pitch']
        msg.yaw = data['stabilizer.yaw']
        msg.thrust = data['stabilizer.thrust']
        self._stabilizer_pub.publish(msg)
