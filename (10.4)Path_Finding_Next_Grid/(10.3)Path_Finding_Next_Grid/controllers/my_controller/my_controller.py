"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

ds_front = robot.getDevice('ds_front')
ds_front.enable(timestep)

ds_front_r = robot.getDevice('ds_front_right')
ds_front_r.enable(timestep)

ds_front_l = robot.getDevice('ds_front_left')
ds_front_l.enable(timestep)

ds_left = robot.getDevice('ds_left')
ds_left.enable(timestep)

ds_right = robot.getDevice('ds_right')
ds_right.enable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
