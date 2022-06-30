""" Decision controller """

import math, struct
from controller import Robot, Motor, Emitter, Receiver

# Main function
if __name__ == "__main__":
    # Create the Robot instance.
    robot = Robot()
    
    time_step = 64
    max_speed = 6
    
    # Define each motor
    leftF_motor = robot.getDevice('leftF')
    leftR_motor = robot.getDevice('leftR')
    rightF_motor = robot.getDevice('rightF')
    rightR_motor = robot.getDevice('rightR')
    
    # Set initial position each motor
    leftF_motor.setPosition(0)
    leftR_motor.setPosition(0)
    rightF_motor.setPosition(0)
    rightR_motor.setPosition(0)
    
    # Set initial speed 
    leftF_motor.setVelocity(0)
    leftR_motor.setVelocity(0)
    rightF_motor.setVelocity(0)
    rightR_motor.setVelocity(0)
    
    emitter = robot.getDevice('emitter')
    
    receiver = robot.getDevice('receiver')
    receiver.enable(time_step)
    
    received = 0
    
    while robot.step(time_step) != -1:
        
        if receiver.getQueueLength()>0 and received != 1:
            comming_message = receiver.getData()
            dataList = struct.unpack("h",comming_message)
            print(dataList)
            received = 1
            
            message = struct.pack("h",2)
            emitter.send(message)
    
        
        
     
