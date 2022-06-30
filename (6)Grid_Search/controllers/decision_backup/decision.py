""" Decision controller """

import math
from controller import Robot, Motor, DistanceSensor, PositionSensor, Compass, InertialUnit

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    time_step = 64
    
    # Define max motor speed
    max_speed = 6
    
    curPos = [[4],[4]]
    
    range = 0
    increment_range = 9.98
    seq_check = 0
    rotState = 0
    
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
    
    # Define distance sensor and enable
    dsFall = robot.getDevice('dsFall')
    dsFall.enable(time_step)
    
    dsFall2 = robot.getDevice('dsFall2')
    dsFall2.enable(time_step)
    
    dsFront = robot.getDevice('dsFront')
    dsFront.enable(time_step)
    
    dsLeft = robot.getDevice('dsLeft')
    dsLeft.enable(time_step)
    
    dsRight = robot.getDevice('dsRight')
    dsRight.enable(time_step)
    
    # Define positional sensor and enable
    ps = robot.getDevice('ps')
    ps.enable(time_step)
    
    # Define compass sensor and enable
    compass = robot.getDevice('compass')
    compass.enable(time_step)
    
    # Define inertial unit and enable
    imu = robot.getDevice('iu')
    imu.enable(time_step)
    
    
    # Main loop:
    while robot.step(time_step) != -1:
        # Set left and right motor speed
        # left_speed = -max_speed
        # right_speed = max_speed
        
        # Take inertial unit values
        imuVal = imu.getRollPitchYaw()
        
        # Take compass sensor values
        compassVal = compass.getValues()
        # print(compassVal)
        
        # Take positional sensor value
        psVal = ps.getValue()
        # Take distance sensors value
        # dsFallVal = dsFall.getValue()
        # dsFrontVal = dsFront.getValue()
        # dsLeftVal = dsLeft.getValue()
        # dsRightVal = dsRight.getValue()        
        
        # Calculate north angle
        angle = math.atan2(compassVal[0], compassVal[2])
        print(angle)        
        
        # Calculate range for next step
        if(seq_check == 0):
            range = psVal + increment_range
            seq_check = 1
        
        # Rotate check
        if(rotState == 1):
            left_speed = max_speed
            right_speed = -max_speed
            range = float('inf')
        else:
            left_speed = max_speed
            right_speed = max_speed
        
        if(psVal > increment_range-0.002 and rotState == 0):
            left_speed = 0
            right_speed = 0
            rotState = 1
            print("girdim")
        
        # Tumble check
        if(imuVal[0]<-3 or imuVal[0]>3):
            left_speed *= -1
            right_speed *= -1
        
        
        # Set motor positions
        leftF_motor.setPosition(range)
        leftR_motor.setPosition(range)
        rightF_motor.setPosition(range)
        rightR_motor.setPosition(range)
        
        # Set motor speeds
        leftF_motor.setVelocity(left_speed)
        leftR_motor.setVelocity(left_speed)
        rightF_motor.setVelocity(right_speed)
        rightR_motor.setVelocity(right_speed)
        
        
        
        pass
    
    # Enter here exit cleanup code.
    