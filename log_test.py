import rospy
import actionlib # rospy simple action api
import signal
from rospy_crazyflie.msg import *
from rospy_crazyflie.srv import *
from crazyflie_client import CrazyflieClient

def log_callback(data, timestamp):
    print("data: %s \ntimestamp: %s" % (data, timestamp))

class TestNode:
    """
    Test connecting to all available crazyflies
    """
    def __init__(self):
        rospy.init_node('testnode')
        self.crazyflies = []
        self.get_crazyflies = rospy.ServiceProxy(
            'crazyflie_server/get_crazyflies',
            GetCrazyflies
        )
        self.get_crazyflies.wait_for_service()

        # Scan for the crazyflies
        response = self.get_crazyflies.call()
        print("crazyflies found %s" % response.crazyflies)
        for name in response.crazyflies:
            self.crazyflies.append(CrazyflieClient(name))

        # Add a log
        name = 'log_stab'
        variables = [
            LogVariable('stabilizer.roll', 'float'),
            LogVariable('stabilizer.pitch', 'float'),
            LogVariable('stabilizer.yaw', 'float')
        ]
        period_ms = 1000
        for crazyflie in self.crazyflies:
            crazyflie.add_log_config(
                name,
                variables,
                period_ms,
                callback=log_callback
            )

    def __delete__(self):
        for crazyflie in self.crazyflies:
            print(crazyflie)
            crazyflie.__delete__()
        self.get_crazyflies.close()

if __name__ == "__main__":
    node = TestNode()
    rospy.on_shutdown(node.__delete__)
    while not rospy.is_shutdown():
        pass
