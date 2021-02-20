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
from .motion_primitive import MotionPrimitive
VELOCITY = 0.2
RATE = 360. / 5.
ANGLE_DEGREES = 360.

class CircleLeft(MotionPrimitive):
    def __init__(self, radius_m, velocity=VELOCITY, angle_degrees=ANGLE_DEGREES):
        super().__init__()
        self.dict['command']='CircleLeft'
        self.dict['radius_m']=radius_m
        self.dict['velocity']=velocity
        self.dict['angle_degrees']=angle_degrees

class CircleRight(MotionPrimitive):
    def __init__(self, radius_m, velocity=VELOCITY, angle_degrees=ANGLE_DEGREES):
        super().__init__(self)
        self.dict['command']='CircleRight'
        self.dict['radius_m']=radius_m
        self.dict['velocity']=velocity
        self.dict['angle_degrees']=angle_degrees

class Back(MotionPrimitive):
    def __init__(self, distance_m, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='Back'
        self.dict['distance_m']=distance_m
        self.dict['velocity']=velocity

class Down(MotionPrimitive):
    def __init__(self, distance_m, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='Down'
        self.dict['distance_m']=distance_m
        self.dict['velocity']=velocity

class Forward(MotionPrimitive):
    def __init__(self, distance_m, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='Forward'
        self.dict['distance_m']=distance_m
        self.dict['velocity']=velocity

class Land(MotionPrimitive):
    def __init__(self, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='Land'
        self.dict['velocity']=velocity

class Left(MotionPrimitive):
    def __init__(self, distance_m, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='Left'
        self.dict['distance_m']=distance_m
        self.dict['velocity']=velocity

class MoveDistance(MotionPrimitive):
    def __init__(self, distance_x_m, distance_y_m, distance_z_m, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='MoveDistance'
        self.dict['distance_x_m']=distance_x_m
        self.dict['distance_y_m']=distance_y_m
        self.dict['distance_z_m']=distance_z_m
        self.dict['velocity']=velocity

class Right(MotionPrimitive):
    def __init__(self, distance_m, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='Right'
        self.dict['distance_m']=distance_m
        self.dict['velocity']=velocity

class TakeOff(MotionPrimitive):
    def __init__(self, height=None, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='TakeOff'
        self.dict['height']=height
        self.dict['velocity']=velocity

class TurnLeft(MotionPrimitive):
    def __init__(self, angle_degrees, rate=RATE):
        super().__init__()
        self.dict['command']='TurnLeft'
        self.dict['angle_degrees']=angle_degrees
        self.dict['rate']=rate

class TurnRight(MotionPrimitive):
    def __init__(self, angle_degrees, rate=RATE):
        super().__init__()
        self.dict['command']='TurnRight'
        self.dict['angle_degrees']=angle_degrees
        self.dict['rate']=rate

class Up(MotionPrimitive):
    def __init__(self, distance_m, velocity=VELOCITY):
        super().__init__()
        self.dict['command']='Up'
        self.dict['distance_m']=distance_m
        self.dict['velocity']=velocity
