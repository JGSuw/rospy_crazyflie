import rospy
import time
import rospy_crazyflie.crazyflie_client as crazyflie_client

if __name__ == "__main__":
    rospy.init_node('rospy_crazyflie_example')

    crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')
    print(crazyflies)
    client = crazyflie_client.CrazyflieClient(crazyflies[0])

    client.take_off(height=.5)
    client.forward(2)
    client.wait()

    del client
