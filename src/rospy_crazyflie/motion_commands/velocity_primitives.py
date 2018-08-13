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

VELOCITY = 0.2
RATE = 360. / 5.

class StartBack:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class SetVelSetpoint:
    def __init__(self, vx, vy, vz, rate_yaw):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.rate_yaw = rate_yaw

class StartBack:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity
class StartCircleLeft:
    def __init__(self, radius_m, velocity = VELOCITY):
        self.radius_m = radius_m
        self.velocity = velocity

class StartCircleRight:
    def __init__(self, radius_m, velocity = VELOCITY):
        self.radius_m = radius_m
        self.velocity = velocity

class StartDown:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartForward:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartLeft:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartLinearMotion:
    def __init__(self, vx, vy, vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz

class StartRight:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class StartTurnLeft:
    def __init__(self, rate = RATE):
        self.rate = rate

class StartTurnRight:
    def __init__(self, rate = RATE):
        self.rate = rate

class StartUp:
    def __init__(self, velocity = VELOCITY):
        self.velocity = velocity

class Stop:
    def __init__(self):
        pass
