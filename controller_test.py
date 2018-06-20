import signal
import time
import rospy
import actionlib # rospy simple action api
from rospy_crazyflie.msg import *
from rospy_crazyflie.srv import *
from crazyflie_client import CrazyflieClient

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

        # Take off
        dt = .1
        vz = .05

        z_setpoint = 0
        for i in range(0, int(.5/vz/dt)):
            for crazyflie in self.crazyflies:
                z_setpoint += vz * dt
                crazyflie.send_hover_setpoint(0, 0, 0, z_setpoint)
            time.sleep(dt)

        for i in range(0, int(8./dt)):
            for crazyflie in self.crazyflies:
                crazyflie.send_hover_setpoint(.25, 0, 360/8., z_setpoint)
            time.sleep(dt)

    def __delete__(self, instance):
        print('foo')
        for crazyflie in self.crazyflies:
            del crazyflie
        self.get_crazyflies.close()


    def __delete__(self):
        for crazyflie in self.crazyflies:
            crazyflie.__delete__()
        self.get_crazyflies.close()

if __name__ == "__main__":
    node = TestNode()
    rospy.on_shutdown(node.__delete__)
    while not rospy.is_shutdown():
        pass
