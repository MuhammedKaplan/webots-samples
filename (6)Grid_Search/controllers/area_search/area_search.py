""" Decision controller """

import math
from controller import Robot, Motor, DistanceSensor, PositionSensor, Compass, InertialUnit

def search_selected_grid(robot):
    # Define the time step 
    time_step = 64
    
    # Define max motor speed
    max_speed = 6
    
    rows, cols = (4, 4)
    grid_map = [[0 for i in range(cols)] for j in range(rows)]
    state_history_length = 16
    state_history_x = [0 for i in range(state_history_length)]
    state_history_y = [0 for i in range(state_history_length)]
    state_history_seq = [20 for i in range(state_history_length)]
    seq_counter = 0
    cur_pos_x = 0
    cur_pos_y = 0
    target_pos_x = 0
    target_pos_y = 0
    target_temp_pos_seq = 0
    
    range = 0
    range1 = 0
    increment_range = 9.98
    movement_check = 0
    rot_state = 0
    tumble_check = 0
    block_check_front = 1
    block_detect_front = 0
    block_check_left = 1
    block_detect_left = 0
    block_check_right = 1
    block_detect_right = 0
    route_found = 0
    rot_angel = 0
    target_found = 0
    range_check = 0
    movement_check = 0
    
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
    ds_fall1 = robot.getDevice('ds_fall1')
    ds_fall1.enable(time_step)
    
    ds_fall2 = robot.getDevice('ds_fall2')
    ds_fall2.enable(time_step)
    
    ds_front = robot.getDevice('ds_front')
    ds_front.enable(time_step)
    
    ds_left = robot.getDevice('ds_left')
    ds_left.enable(time_step)
    
    ds_right = robot.getDevice('ds_right')
    ds_right.enable(time_step)
    
    # Define positional sensor and enable
    ps = robot.getDevice('ps')
    ps.enable(time_step)
    
    ps1 = robot.getDevice('ps1')
    ps1.enable(time_step)
    
    # Define compass sensor and enable
    compass = robot.getDevice('compass')
    compass.enable(time_step)
    
    # Define inertial unit and enable
    imu = robot.getDevice('iu')
    imu.enable(time_step)   
    
    test()
    print("Test Function Main Code:")
    
    
    # Main loop:
    while robot.step(time_step) != -1:
        test()
        print("Test Function While Loop")
        
        # Take inertial unit values
        imu_value = imu.getRollPitchYaw()
        
        # Take compass sensor values
        compass_value = compass.getValues()
        # print(compassVal)
        
        # Take positional sensor value
        ps_value = ps.getValue()
        ps1_value = ps1.getValue()
        # Take distance sensors value
        ds_fall1_value = ds_fall1.getValue()
        ds_fall2_value = ds_fall2.getValue()
        ds_front_value = ds_front.getValue()
        ds_left_value = ds_left.getValue()
        ds_right_value = ds_right.getValue() 
        
        # Calculate north angle
        # print(compass_value)
        north = math.atan2(compass_value[0], compass_value[2])
        # print(north)       s            
        
        # If robot discover undiscovered area then update map
        if(grid_map[cur_pos_x][cur_pos_y] == 0):
        
            grid_map[cur_pos_x][cur_pos_y] = 1
            state_history_x[seq_counter] = cur_pos_x
            state_history_y[seq_counter] = cur_pos_y
            state_history_seq[seq_counter] = seq_counter
            seq_counter += 1
            # Print updated map
            i = rows - 1            
            while(i >= 0):
                print(grid_map[i])
                i -= 1
            print("************")
        
        # Block detection
        if(ds_front_value < 1535 and block_check_front == 1):
            block_detect_front = 1
            
            if(-0.01<north<0.01):
                grid_map[cur_pos_x + 1][cur_pos_y] = 3
            
            if(1.56<north<1.58):
                grid_map[cur_pos_x][cur_pos_y + 1] = 3
            
            if(-3.15<north<-3.13 or 3.13<north<3.15):
                grid_map[cur_pos_x - 1][cur_pos_y] = 3
            
            if(-1.58<north<-1.56):
                grid_map[cur_pos_x][cur_pos_y - 1] = 3
                
        else:
            block_check_front = 0 
        
        if(target_found == 0):
            # Target grid search
            if(cur_pos_x + 1 < rows):
                if(grid_map[cur_pos_x + 1][cur_pos_y] == 0):
                    target_pos_x = cur_pos_x + 1
                    target_pos_y = cur_pos_y
                    target_pos_direction = "north"
                    target_found = 1
            
            if(cur_pos_y + 1 < rows):
                if(grid_map[cur_pos_x][cur_pos_y + 1] == 0 and target_found == 0):
                    target_pos_x = cur_pos_x
                    target_pos_y = cur_pos_y + 1
                    target_pos_direction = "east"
                    target_found = 1
            
            if(cur_pos_x - 1 >= 0):            
                if(grid_map[cur_pos_x - 1][cur_pos_y] == 0 and target_found == 0):
                    target_pos_x = cur_pos_x - 1
                    target_pos_y = cur_pos_y
                    target_pos_direction = "south"
                    target_found = 1
            
            if(cur_pos_y - 1 >= 0):            
                if(grid_map[cur_pos_x][cur_pos_y - 1] == 0 and target_found == 0):
                    target_pos_x = cur_pos_x
                    target_pos_y = cur_pos_y - 1
                    target_pos_direction = "west"
                    target_found = 1
                        
            # if(grid_map[cur_pos_x + 1][cur_pos_y + 1] == 0 and target_found == 0):
                # target_pos_x = cur_pos_x + 1
                # target_pos_y = cur_pos_y + 1
                # target_pos_direction = "northEast"
                # target_found = 1
                        
            # if(grid_map[cur_pos_x - 1][cur_pos_y + 1] == 0 and target_found == 0):
                # target_pos_x = cur_pos_x - 1
                # target_pos_y = cur_pos_y + 1
                # target_pos_direction = "southEast"
                # target_found = 1
                            
            # if(grid_map[cur_pos_x - 1][cur_pos_y - 1] == 0 and target_found == 0):
                # target_pos_x = cur_pos_x - 1
                # target_pos_y = cur_pos_y - 1
                # target_pos_direction = "southWest"
                # target_found = 1
            
            # if(grid_map[cur_pos_x + 1][cur_pos_y - 1] == 0 and target_found == 0):
                # target_pos_x = cur_pos_x + 1
                # target_pos_y = cur_pos_y - 1
                # target_pos_direction = "northWest"
                # target_found = 1
        
        
        if(target_found == 0):
            # Return max State History Sequence
            state_history_maxSeq = 0
            brk = 0
            while(state_history_maxSeq < state_history_length and brk == 0):
                if(state_history_seq[state_history_maxSeq] > 16):
                    brk = 1
                else:
                    state_history_maxSeq += 1
            seq = 0
            print(state_history_maxSeq)
            while(state_history_maxSeq >= 0):
                check_x = state_history_x[state_history_maxSeq]
                check_y = state_history_y[state_history_maxSeq]
                
                if(grid_map[check_x + 1][check_y] == 0):
                    target_temp_pos_seq = state_history_maxSeq
                    target_temp_pos_direction = "north"
                    route_found = 1
                
                if(grid_map[check_x][check_y + 1] == 0 and route_found == 0):
                    target_temp_pos_seq = state_history_maxSeq
                    target_temp_pos_direction = "east"
                    route_found = 1
                            
                if(grid_map[check_x - 1][check_y] == 0 and route_found == 0):
                    target_temp_pos_seq = state_history_maxSeq
                    target_temp_pos_direction = "south"
                    route_found = 1
                            
                if(grid_map[check_x][check_y - 1] == 0 and route_found == 0):
                    target_temp_pos_seq = state_history_maxSeq
                    target_temp_pos_direction = "west"
                    route_found = 1
            

        # Calculate range for next step
        if(target_found == 1 and range_check == 0):
            if((target_pos_direction == "north" and -0.01<north<0.01)
            or (target_pos_direction == "east" and 1.56<north<1.58)
            or (target_pos_direction == "south" and (-3.15<north<-3.13 or 3.13<north<3.15))
            or (target_pos_direction == "west" and -1.58<north<-1.56)):
                range = ps_value + increment_range
                range1 = ps1_value + increment_range
                range_check = 1
                movement_check = 1
                print("Calculate range for next step")
            
            elif(target_pos_direction == "north"):
                rot_angle = 0
                rot_state = 1
                range_check = 1
                
            elif(target_pos_direction == "east"):
                rot_angle = 1
                rot_state = 1
                range_check = 1
            
            elif(target_pos_direction == "south"):
                rot_angle = 2
                rot_state = 1
                range_check = 1
            
            elif(target_pos_direction == "west"):
                rot_angle = 3
                rot_state = 1
                range_check = 1
        
        
        
        # Rotate check
        if(rot_state == 1):
            left_speed = max_speed/16
            right_speed = -max_speed/16
            
            range = float('inf')
            range1 = float('inf')
            if(rot_angle == 0):
                # North
                if(-0.01<north<0.01):
                    rot_state = 0
                    left_speed = 0
                    right_speed = 0
                    range_check = 0
                    
            if(rot_angle == 1):
                # East
                if(1.56<north<1.58):
                    rot_state = 0
                    left_speed = 0
                    right_speed = 0
                    range_check = 0
                    
            if(rot_angle == 2):
                # South
                if(-3.15<north<-3.13 or 3.13<north<3.15):
                    rot_state = 0
                    left_speed = 0
                    right_speed = 0
                    range_check = 0
                    
            if(rot_angle == 3):
                # West
                if(-1.58<north<-1.56):
                    rot_state = 0
                    left_speed = 0
                    right_speed = 0
                    range_check = 0
            
        else:
            left_speed = max_speed
            right_speed = max_speed
        
        
        # Tumble check
        if((imu_value[0]<-3 or imu_value[0]>3) and tumble_check == 0):
            left_speed *= -1
            right_speed *= -1
            tumble_check = 1
        if(not(imu_value[0]<-3 or imu_value[0]>3) and tumble_check == 1):
            left_speed *= -1
            right_speed *= -1
            tumble_check = 0
        
        # Rotate position check
        if(ps_value > range-0.002 and movement_check == 1):
            left_speed = 0
            right_speed = 0
            cur_pos_x = target_pos_x
            cur_pos_y = target_pos_y
            target_found = 0
            range_check = 0
            movement_check = 0
            print("Rotate position check")
        
        # Set motor positions
        leftF_motor.setPosition(range)
        leftR_motor.setPosition(range)
        rightF_motor.setPosition(range1)
        rightR_motor.setPosition(range1)
        
        # Set motor speeds
        leftF_motor.setVelocity(left_speed)
        leftR_motor.setVelocity(left_speed)
        rightF_motor.setVelocity(right_speed)
        rightR_motor.setVelocity(right_speed)
        
        pass

def test():
    print("Hello This is Function")

if __name__ == "__main__":
    # Create the Robot instance.
    robot = Robot()
    
    search_selected_grid(robot)
    
    
    
    
        
        
     

    
    
    
    
    
    
    
    
    
    