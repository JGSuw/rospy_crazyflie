#! /usr/bin/env python
from crazyflie_server import CrazyflieServer
if __name__ == "__main__":
    node = CrazyflieServer()
    node.run()
