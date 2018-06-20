#! /usr/bin/python
import sys
import time
import threading
import rospy
import actionlib

from crazyflie_log import CrazyflieLog
from crazyflie_control import CrazyflieControl
import cflib.crtp

from cflib.crazyflie import Crazyflie
from rospy_crazyflie.srv import GetCrazyflies, GetCrazyfliesResponse, Disconnect
from rospy_crazyflie.msg import ConnectAction, ConnectActionGoal

class CrazyflieServer:

    def __init__(self):
        # Ros stuff
        rospy.init_node("crazyflie_server")

        # Dictionaries to hold the crazyflies and related objects
        # self._address = rospy.get_param('~address')
        self._uris = rospy.get_param('~uris')
        print(self._uris)
        self._crazyflies = {}
        self._crazyflie_logs = {}
        self._controllers = {}

        cflib.crtp.init_drivers(enable_debug_driver=False)

        self._get_crazyflies_srv = rospy.Service (
            '~get_crazyflies',
            GetCrazyflies,
            self._get_crazyflies_cb
        )

        self._disconnect_srv = rospy.Service (
            '~disconnect',
            Disconnect,
            self._disconnect_srv_cb
        )

        for uri in self._uris:
            parts = uri.split('/')
            channel = parts[3]
            name = 'cf%s' % channel
            cf = Crazyflie()
            cf.connected.add_callback(self._connected)
            cf.disconnected.add_callback(self._disconnected)
            cf.connection_failed.add_callback(self._connection_failed)
            cf.connection_lost.add_callback(self._connection_lost)
            cf.open_link(uri)
            self._crazyflies[uri] = (name, cf)

    def _get_crazyflies_cb (self, request) :
        response = GetCrazyfliesResponse()
        for (name, crazyflie) in self._crazyflies.values():
            if crazyflie.is_connected():
                response.crazyflies.append(name)
        return response


    def _connected(self, link_uri):
        print('Connected to %s.' % (link_uri))
        if link_uri not in self._crazyflie_logs:
            log = CrazyflieLog(*self._crazyflies[link_uri])
            self._crazyflie_logs[link_uri] = log
        if link_uri not in self._controllers:
            controller = CrazyflieControl(*self._crazyflies[link_uri])
            self._controllers[link_uri] = controller

    def _connection_failed(self, link_uri, msg):
        """Callback when initial connection fails (i.e. no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        #self._crazyflies[link_uri][1].open_link(link_uri)


    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e.
        Crazyflie moves out of range)"""
        print('Connection to %s lost : %s' % (link_uri, msg))
        #self._crazyflies[link_uri][1].open_link(link_uri)


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        print('Reconnecting...')
        #self._crazyflies[link_uri][1].open_link(link_uri)

    def _disconnect_srv_cb(self, request):
        for link_uri in self._crazyflies.keys():
            (name, crazyflie) = self._crazyflies[link_uri]
            if name == request.name:
                self._crazyflie_logs[link_uri].stop_logs()
        return []

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(.1)
        print("shutting down")
        for link_uri in self._crazyflies.keys():
            cf = self._crazyflies[link_uri][1]
            print("closing link to %s" % link_uri)
            if cf.is_connected():
                try:
                    self._controllers[link_uri].land()
                except:
                    pass
                cf.close_link()
                del(cf)
