"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

ds_front_right = robot.getDevice('ds_front_right')
ds_front_right.enable(timestep)

ds_front_left = robot.getDevice('ds_front_left')
ds_front_left.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    dsfr_val = ds_front_right.getValue()
    dsfl_val = ds_front_left.getValue()
    
    print("Right: ", dsfr_val)
    print("Left: ", dsfl_val)
    print("------------------")
    
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
