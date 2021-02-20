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

class StartBack(MotionPrimitive):
    def __init__(self, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartBack'
        self.dict['velocity']=velocity

class SetVelSetpoint(MotionPrimitive):
    def __init__(self, vx, vy, vz, rate_yaw):
        super().__init__()
        self.dict['command']='SetVelSetpoint'
        self.dict['vx']=vx
        self.dict['vy']=vy
        self.dict['vz']=vz
        self.dict['rate_yaw']=rate_yaw

class StartBack(MotionPrimitive):
    def __init__(self, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartBack'
        self.dict['velocity']=velocity

class StartCircleLeft(MotionPrimitive):
    def __init__(self, radius_m, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartCircleLeft'
        self.dict['radius_m']=radius_m
        self.dict['velocity']=velocity

class StartCircleRight(MotionPrimitive):
    def __init__(self, radius_m, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartCircleRight'
        self.dict['radius_m']=radius_m
        self.dict['velocity']=velocity

class StartDown(MotionPrimitive):
    def __init__(self, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartDown'
        self.dict['velocity']=velocity

class StartForward(MotionPrimitive):
    def __init__(self, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartForward'
        self.dict['velocity']=velocity

class StartLeft(MotionPrimitive):
    def __init__(self, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartLeft'
        self.dict['velocity']=velocity

class StartLinearMotion(MotionPrimitive):
    def __init__(self, vx, vy, vz):
        super().__init__()
        self.dict['command']='StartLinearMotion'
        self.dict['vx']=vx
        self.dict['vy']=vy
        self.dict['vz']=vz

class StartRight(MotionPrimitive):
    def __init__(self, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartRight'
        self.dict['velocity']=velocity

class StartTurnLeft(MotionPrimitive):
    def __init__(self, rate = RATE):
        super().__init__()
        self.dict['command']='StartTurnLeft'
        self.dict['rate']=rate

class StartTurnRight(MotionPrimitive):
    def __init__(self, rate = RATE):
        super().__init__()
        self.dict['command']='StartTurnRight'
        self.dict['rate']=rate

class StartUp(MotionPrimitive):
    def __init__(self, velocity = VELOCITY):
        super().__init__()
        self.dict['command']='StartUp'
        self.dict['velocity']=velocity

class Stop(MotionPrimitive):
    def __init__(self):
        super().__init__()
        self.dict['command']='Stop'
        pass
