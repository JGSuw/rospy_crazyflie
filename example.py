import rospy
import time
from rospy_crazyflie.crazyflie_client import CrazyflieClient
from rospy_crazyflie.srv import *

if __name__ == "__main__":
    rospy.init_node('rospy_crazyflie_example')

    get_crazyflies = rospy.ServiceProxy(
        'crazyflie_server/get_crazyflies',
        GetCrazyflies
    )
    get_crazyflies.wait_for_service()
    response = get_crazyflies()
    for crazyflie_name in response.crazyflies:
        print(crazyflie_name)

    client = CrazyflieClient('cf85')

    client.take_off(height=.5)
    client.forward(2)
    client.wait()

    del client
