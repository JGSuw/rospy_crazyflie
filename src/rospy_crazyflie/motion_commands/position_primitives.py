VELOCITY = 0.2
RATE = 360. / 5.
ANGLE_DEGREES = 360.

class Back:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class CircleLeft:
    def __init__(self, radius_m, velocity=VELOCITY, angle_degrees=ANGLE_DEGREES):
        self.radius_m = radius_m
        self.velocity = velocity
        self.angle_degrees = angle_degrees

class CircleRight:
    def __init__(self, radius_m, velocity=VELOCITY, angle_degrees=ANGLE_DEGREES):
        self.radius_m = radius_m
        self.velocity = velocity
        self.angle_degrees = angle_degrees

class Down:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class Forward:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class Land:
    def __init__(self, velocity=VELOCITY):
        self.velocity = velocity

class Left:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class MoveDistance:
    def __init__(self, distance_x_m, distance_y_m, distance_z_m, velocity=VELOCITY):
        self.x = distance_x_m
        self.y = distance_y_m
        self.z = distance_z_m
        self.velocity = velocity
class Right:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity

class TakeOff:
    def __init__(self, height=None, velocity=VELOCITY):
        self.height = height
        self.velocity = velocity

class TurnLeft:
    def __init__(self, angle_degrees, rate=RATE):
        self.angle_degrees = angle_degrees
        self.rate = rate

class TurnRight:
    def __init__(self, angle_degrees, rate=RATE):
        self.angle_degrees = angle_degrees
        self.rate = rate

class Up:
    def __init__(self, distance_m, velocity=VELOCITY):
        self.distance_m = distance_m
        self.velocity = velocity
