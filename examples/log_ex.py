# Necessary imports
import rospy
from rospy_crazyflie.crazyflie_client import CrazyflieClient
from rospy_crazyflie.msg import LogVariable

rospy.init_node('example', log_level=rospy.DEBUG)
rospy.logdebug("getting crazyflie...")
client = CrazyflieClient('/crazyflie1')
rospy.logdebug("got crazyflie")

# Define the callback function
# @data dictionary associating variable names with values
# @timestamp crazyflie system tick count at time the log was sent
def log_callback(data, timestamp):
    print("data: {}".format(data))
    print("timestamp: {}".format(timestamp))

# Creating the log config
config_name = 'custom_log_config'
config_vars = [
    LogVariable('stabilizer.roll', 'float'),
    LogVariable('stabilizer.pitch', 'float'),
    LogVariable('stabilizer.yaw', 'float'),
    LogVariable('kalman.stateX', 'float'),
    LogVariable('kalman.stateY', 'float'),
    LogVariable('kalman.stateZ', 'float')
]
period_in_ms = 100 # sends this log 10 times per second
client.add_log_config(config_name, config_vars, period_in_ms, callback=log_callback)
