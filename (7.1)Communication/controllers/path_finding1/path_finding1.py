import math, collections, struct
from controller import Robot, Motor, DistanceSensor, PositionSensor, Compass, InertialUnit, Emitter, Receiver


def grid_search(robot):
    # Define the time step 
    grid_search.time_step = 64
    grid_search.robot = robot
    # Define max motor speed
    grid_search.max_speed = 6
    
    grid_search.rows = 8
    grid_search.cols = 8
    grid_search.wall = 3
    grid_search.visit = 1
    grid_search.goal = 0
    grid_search.grid_map = [[0 for i in range(grid_search.cols)] for j in range(grid_search.rows)]
    
    grid_search.cur_pos_x = 0
    grid_search.cur_pos_y = 1  
        
    grid_search.increment_range = 9.98
    grid_search.movement_check = 0
    route_found = 0
    
    grid_search.rot_angle = 0
    grid_search.rot_state = 0
    
    grid_search.tumble_state = 0
    
    grid_search.exit_flag = 0
    
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
    ds_fall1.enable(grid_search.time_step)
    
    ds_fall2 = robot.getDevice('ds_fall2')
    ds_fall2.enable(grid_search.time_step)
    
    ds_front = robot.getDevice('ds_front')
    ds_front.enable(grid_search.time_step)
    
    ds_left = robot.getDevice('ds_left')
    ds_left.enable(grid_search.time_step)
    
    ds_right = robot.getDevice('ds_right')
    ds_right.enable(grid_search.time_step)
    
    # Define positional sensor and enable
    ps_left = robot.getDevice('ps_left')
    ps_left.enable(grid_search.time_step)
    
    ps_right = robot.getDevice('ps_right')
    ps_right.enable(grid_search.time_step)
    
    # Define compass sensor and enable
    compass = robot.getDevice('compass')
    compass.enable(grid_search.time_step)
    
    # Define inertial unit and enable
    imu = robot.getDevice('iu')
    imu.enable(grid_search.time_step)   
    
    # Define cominication units
    grid_search.emitter = robot.getDevice('emitter')
    
    grid_search.receiver = robot.getDevice('receiver')
    grid_search.receiver.enable(grid_search.time_step)
     
    # Main loop:
    while robot.step(grid_search.time_step) != -1:
        # Take inertial unit values
        grid_search.imu_value = imu.getRollPitchYaw()
        
        # Take compass sensor values
        compass_value = compass.getValues()
        
        # Take positional sensor value
        grid_search.ps_left_value = ps_left.getValue()
        grid_search.ps_right_value = ps_right.getValue()
        
        # Take distance sensors value
        ds_fall1_value = ds_fall1.getValue()
        ds_fall2_value = ds_fall2.getValue()
        grid_search.ds_front_value = ds_front.getValue()
        grid_search.ds_left_value = ds_left.getValue()
        grid_search.ds_right_value = ds_right.getValue() 
        
        # Calculate north angle
        grid_search.north = math.atan2(compass_value[0], compass_value[2])                
             
        # Map update
        update_map()
        
        
        
        if grid_search.receiver.getQueueLength()>0:
            
            comming_message_byte = grid_search.receiver.getData()
            message_byte = struct.unpack("h",comming_message_byte)
            grid_search.receiver.nextPacket()
            if(message_byte[0] > 0):
                comming_message = grid_search.receiver.getData()
                dataList = struct.unpack("h"*message_byte[0],comming_message)
                # print(dataList)
                
                
                for i in range(0, len(dataList), 3):
                    grid_search.update_pos_val = dataList[i]
                    grid_search.update_pos_x = dataList[i+1]
                    grid_search.update_pos_y = dataList[i+2]
                    grid_search.grid_map[grid_search.update_pos_x][grid_search.update_pos_y] = grid_search.update_pos_val
                
                print("mesej alindi")
                draw_map(grid_search.grid_map)
                
                # if(dataList[0] == 1):
                #     grid_search.r1_curX = dataList[1]
                #     grid_search.r1_curY = dataList[2]
                #     grid_search.r1_targetX = dataList[4]
                #     grid_search.r1_targetY = dataList[5]
                    
                # if(dataList[0] == 2):
                #     grid_search.r2_curX = dataList[1]
                #     grid_search.r2_curY = dataList[2]
                #     grid_search.r2_targetX = dataList[4]
                #     grid_search.r2_targetY = dataList[5]
                    
                # if(dataList[0] == 3):
                #     grid_search.r3_curX = dataList[1]
                #     grid_search.r3_curY = dataList[2]
                #     grid_search.r3_targetX = dataList[4]
                #     grid_search.r3_targetY = dataList[5]
                
                
                
            else:
                comming_message = grid_search.receiver.getData()
                dataList = struct.unpack("hhh",comming_message)
                                
                grid_search.grid_map[dataList[1]][dataList[2]] = dataList[0]
            
            grid_search.receiver.nextPacket()
            
        
        # Find target position
        target_pos_search()        
        
        # Calculate range for next step
        cal_next_step_range()        
        
        # Rotate check
        rotate_check()        
        
        # Tumble check
        tumble_check()
        
        # Rotate position check
        rotate_pos_check()
        
        # Set motor positions
        leftF_motor.setPosition(cal_next_step_range.rangeLeft)
        leftR_motor.setPosition(cal_next_step_range.rangeLeft)
        rightF_motor.setPosition(cal_next_step_range.rangeRight)
        rightR_motor.setPosition(cal_next_step_range.rangeRight)
        
        # Set motor speeds
        leftF_motor.setVelocity(grid_search.left_speed)
        leftR_motor.setVelocity(grid_search.left_speed)
        rightF_motor.setVelocity(grid_search.right_speed)
        rightR_motor.setVelocity(grid_search.right_speed)
        
        
        
        pass
    
def draw_map(print_map):
    # Print updated map
    rows = len(print_map)
    cols = len(print_map[0])
    # print(rows,cols)
    
    for i in range(rows - 1, -1, -1):
        print(print_map[i])
    print("***"*rows)
    
def update_map():
# =============================================================================
# This function update map according to new datas
# =============================================================================
        
    # If robot discover undiscovered area then update map
    if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] == 0):
    
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 5
        
        draw_map(grid_search.grid_map)
        

def target_pos_search():
# =============================================================================
# This function find the new target position on map
# =============================================================================
    if target_pos_search.found == 0 and target_pos_search.target_found == 0 and target_pos_search.sng_state == 0:    
        # print("girdim")
        # print(grid_search.cur_pos_x, grid_search.cur_pos_y)
        # List boundary control
        if(grid_search.cur_pos_x + 1 < grid_search.rows):
            # print("out of list north")
            if(grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0):
                target_pos_search.x = grid_search.cur_pos_x + 1
                target_pos_search.y = grid_search.cur_pos_y
                target_pos_search.direction = "north"
                target_pos_search.found = 1
                target_pos_search.target_found = 1
                target_pos_search.message_send = 1
                # print("north")
                # print("-----------------------")
        
        
        # List boundary control
        if(grid_search.cur_pos_y + 1 < grid_search.cols):
            # print("out of list east")
            if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0 and target_pos_search.found == 0):
                target_pos_search.x = grid_search.cur_pos_x
                target_pos_search.y = grid_search.cur_pos_y + 1
                target_pos_search.direction = "east"
                target_pos_search.found = 1
                target_pos_search.target_found = 1
                target_pos_search.message_send = 1
                # print("east")
        
        # List boundary control
        if(grid_search.cur_pos_x - 1 >= 0):            
            if(grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0 and target_pos_search.found == 0):
                target_pos_search.x = grid_search.cur_pos_x - 1
                target_pos_search.y = grid_search.cur_pos_y
                target_pos_search.direction = "south"
                target_pos_search.found = 1
                target_pos_search.target_found = 1 
                target_pos_search.message_send = 1
        
        # List boundary control
        if(grid_search.cur_pos_y - 1 >= 0):            
            if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0 and target_pos_search.found == 0):
                target_pos_search.x = grid_search.cur_pos_x
                target_pos_search.y = grid_search.cur_pos_y - 1
                target_pos_search.direction = "west"
                target_pos_search.found = 1
                target_pos_search.target_found = 1
                target_pos_search.message_send = 1
    
    if(target_pos_search.sng_state != 1 and target_pos_search.found == 0 and target_pos_search.target_found == 0):
        target_pos_search.path = search_near_grid(grid_search.grid_map, (grid_search.cur_pos_x, grid_search.cur_pos_y))
        # print(target_pos_search.path)
        if (target_pos_search.path != None):
            # Search near grid state
            target_pos_search.sng_state = 1
            target_pos_search.target_found = 1
            target_pos_search.sng_flag = 1 
            target_pos_search.message_send = 1
            x, y = target_pos_search.path[-1]
            message = struct.pack("hhh",1,x,y)
            byte = -1
            byte_message = struct.pack("h",byte)
            grid_search.emitter.send(byte_message)
            grid_search.emitter.send(message)
            
            print("sng check")
            
    if(target_pos_search.sng_state == 1):
        if(target_pos_search.sng_flag == 1):
            if(target_pos_search.path):
                target_pos_search.x, target_pos_search.y = target_pos_search.path.pop(0)  
                target_pos_search.sng_flag = 0
                # print(target_pos_search.x, target_pos_search.y)
            else:
                target_pos_search.sng_state = 0
                target_pos_search.target_found = 0
                # print("sng_state 0")
                
        if(target_pos_search.sng_flag == 0):
            if(target_pos_search.x == grid_search.cur_pos_x and target_pos_search.y == grid_search.cur_pos_y):
                target_pos_search.sng_flag = 1
            
            if(target_pos_search.x == grid_search.cur_pos_x + 1 and target_pos_search.y == grid_search.cur_pos_y):
                target_pos_search.direction = "north"
            
            if(target_pos_search.x == grid_search.cur_pos_x and target_pos_search.y == grid_search.cur_pos_y + 1):
                target_pos_search.direction = "east"
                
            if(target_pos_search.x == grid_search.cur_pos_x - 1 and target_pos_search.y == grid_search.cur_pos_y):
                target_pos_search.direction = "south"
                # print("south")
                
            if(target_pos_search.x == grid_search.cur_pos_x and target_pos_search.y == grid_search.cur_pos_y - 1):
                target_pos_search.direction = "west"
    
    if(target_pos_search.message_send == 1):
        grid_search.grid_map[target_pos_search.x][target_pos_search.y] = 5
        byte = 3
        message = struct.pack("hhh",1,grid_search.cur_pos_x, grid_search.cur_pos_y)
        for j in range(grid_search.cols):
            for i in range(grid_search.rows):
                if(grid_search.grid_map[i][j] == 1 or grid_search.grid_map[i][j] == 5 
                   or grid_search.grid_map[i][j] == 3):
                    message += struct.pack("hhh",grid_search.grid_map[i][j],i,j)
                    byte += 3
        
        byte_message = struct.pack("h",byte)
        grid_search.emitter.send(byte_message)
        grid_search.emitter.send(message)
        target_pos_search.message_send = 0                      

def search_near_grid(grid_map, start):
    queue = collections.deque([[start]])
    seen = set([start])
        
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        if grid_map[x][y] == grid_search.goal:
            return path
        for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
            if 0 <= x2 < grid_search.rows and 0 <= y2 < grid_search.cols and grid_map[x2][y2] != grid_search.wall and (x2, y2) not in seen:
                queue.append(path + [(x2, y2)])
                seen.add((x2, y2))
        
def cal_next_step_range():
# =============================================================================
# This function calculate next step range value or 
# decides turn direction to target position direction
# =============================================================================
    
    # If target position found and range not calculated before
    if((target_pos_search.found == 1 or target_pos_search.sng_state == 1) and cal_next_step_range.state == 0):
                
        # If target direction same to robot directon 
        if((target_pos_search.direction == "north" and -0.01<grid_search.north<0.01)
            or (target_pos_search.direction == "east" and 1.56<grid_search.north<1.58)
            or (target_pos_search.direction == "south" and (-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15))
            or (target_pos_search.direction == "west" and -1.58<grid_search.north<-1.56)):
                # Left or right wheel range is current positional sensor value + increment value
                cal_next_step_range.rangeLeft = grid_search.ps_left_value + grid_search.increment_range
                cal_next_step_range.rangeRight = grid_search.ps_right_value + grid_search.increment_range
                
                cal_next_step_range.state = 1
                grid_search.movement_check = 1
                # print("Calculate range for next step")
                
        # Not same and if target direction is north then turn north
        elif(target_pos_search.direction == "north"):
            grid_search.rot_angle = 0
            grid_search.rot_state = 1
            cal_next_step_range.state = 1
            
        # Not same and if target direction is north then turn east
        elif(target_pos_search.direction == "east"):
            grid_search.rot_angle = 1
            grid_search.rot_state = 1
            cal_next_step_range.state = 1
        
        # Not same and if target direction is north then turn south
        elif(target_pos_search.direction == "south"):
            grid_search.rot_angle = 2
            grid_search.rot_state = 1
            cal_next_step_range.state = 1
            # print("turning south")
        
        # Not same and if target direction is north then turn west
        elif(target_pos_search.direction == "west"):
            grid_search.rot_angle = 3
            grid_search.rot_state = 1
            cal_next_step_range.state = 1

def rotation_direction(angle):
# =============================================================================
# This function detect rotate direction according to the direction of the return
# =============================================================================
    
    # Direction of return is north
    if(angle == 0):
        if(grid_search.north > 0):
            grid_search.right_speed = grid_search.max_speed/18
            grid_search.left_speed = -grid_search.max_speed/18
        else:
            grid_search.right_speed = -grid_search.max_speed/18
            grid_search.left_speed = grid_search.max_speed/18
    
    # Direction of return is east
    if(angle == 1):
        if(-1.57 < grid_search.north < 1.57):
            grid_search.right_speed = -grid_search.max_speed/18
            grid_search.left_speed = grid_search.max_speed/18
        else:
            grid_search.right_speed = grid_search.max_speed/18
            grid_search.left_speed = -grid_search.max_speed/18
    
    # Direction of return is south     
    if(angle == 2):
        if(0 < grid_search.north < 3.14):
            grid_search.right_speed = -grid_search.max_speed/18
            grid_search.left_speed = grid_search.max_speed/18
        else:
            grid_search.right_speed = grid_search.max_speed/18
            grid_search.left_speed = -grid_search.max_speed/18
    
    # Direction of return is west
    if(angle == 3):
        if(-1.57 < grid_search.north < 1.57):
            grid_search.right_speed = grid_search.max_speed/18
            grid_search.left_speed = -grid_search.max_speed/18
        else:
            grid_search.right_speed = -grid_search.max_speed/18
            grid_search.left_speed = grid_search.max_speed/18

def rotate_check():
# =============================================================================
# This function controls the rotational status and 
# decides when it will stop according to the rotation angle and 
# if the robot is not in the case of rotation, it should go forward.
# =============================================================================
    
    # If robot come to rotate state
    if(grid_search.rot_state == 1):
                       
        # Define left and right range to infinite
        cal_next_step_range.rangeLeft = float('inf')
        cal_next_step_range.rangeRight = float('inf')
        
        # If rotate angle is 0
        if(grid_search.rot_angle == 0):
            
            # Rotation direction according to target rotation direction
            rotation_direction(0) 
                           
            # Robot then turn north
            if(-0.01<grid_search.north<0.01):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
        
        # If rotate angle is 1
        if(grid_search.rot_angle == 1):
            
            # Rotation direction according to target rotation direction
            rotation_direction(1)
            
            # Robot then turn east
            if(1.56<grid_search.north<1.58):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
        
        # If rotate angle is 2
        if(grid_search.rot_angle == 2):
            
            # Rotation direction according to target rotation direction
            rotation_direction(2)
            
            # Robot then turn south
            if(-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
        
        # If rotate angle is 3
        if(grid_search.rot_angle == 3):
            
            # Rotation direction according to target rotation direction
            rotation_direction(3)
            
            # Robot then turn west
            if(-1.58<grid_search.north<-1.56):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
    # Else go forward
    else:
        if(grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 4
           or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 6
           or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 7):
            grid_search.left_speed = 0
            grid_search.right_speed = 0
        else:
            grid_search.left_speed = grid_search.max_speed
            grid_search.right_speed = grid_search.max_speed

def tumble_check():
# =============================================================================
# This function check the robot tumble status and 
# decides when reverse the wheel speed direction to the robot inertial unit 
# sensor values
# =============================================================================
    if((grid_search.imu_value[0]<-3 or grid_search.imu_value[0]>3) and grid_search.tumble_state == 0):
        grid_search.left_speed *= -1
        grid_search.right_speed *= -1
        grid_search.tumble_state = 1
    if(not(grid_search.imu_value[0]<-3 or grid_search.imu_value[0]>3) and grid_search.tumble_state == 1):
        grid_search.left_speed *= -1
        grid_search.right_speed *= -1
        grid_search.tumble_state = 0

def rotate_pos_check():
# =============================================================================
# This function check robot come to target position and reset some values and
# check any block in around the robot
# =============================================================================
    # If robot go to the target position 
    if(grid_search.ps_left_value > cal_next_step_range.rangeLeft-0.002 and grid_search.movement_check == 1):
        # Reset variables
        grid_search.left_speed = 0
        grid_search.right_speed = 0        
        target_pos_search.found = 0
        cal_next_step_range.state = 0
        grid_search.movement_check = 0
        target_pos_search.direction = ""
        
        print("reset")
        if(target_pos_search.sng_state == 1):
            if(target_pos_search.sng_flag == 0):
                target_pos_search.sng_flag = 1
        else:
            target_pos_search.target_found = 0
        # print("rotate pos check")
        
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 1
        
        # Now robot current position is target postion
        grid_search.cur_pos_x = target_pos_search.x
        grid_search.cur_pos_y = target_pos_search.y
        
        
        # print(grid_search.cur_pos_x)
        # print(grid_search.cur_pos_y)
        
        # Check front, left or right sight of robot for any block
        block_check()
        
def block_check():    
    # Front Block detection
    if(grid_search.ds_front_value < 1535):
        # If direction of robot is north
        if((-0.01<grid_search.north<0.01) and 
           grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0):
            grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3
        # If direction of robot is east
        if((1.56<grid_search.north<1.58) and 
           grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0):
            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
        
        # If direction of robot is south
        if((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and 
           grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0):
            grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
        
        # If direction of robot is west
        if((-1.58<grid_search.north<-1.56) and 
           grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0):
            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
    
    # Right Block detection
    if(grid_search.ds_right_value < 1535):
        
        # If direction of robot is north
        if((-0.01<grid_search.north<0.01) and 
           grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0):
            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
        # If direction of robot is east
        if((1.56<grid_search.north<1.58) and 
           grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0):
            grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
        # If direction of robot is south
        if((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and 
           grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0):
            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3    
        # If direction of robot is west
        if((-1.58<grid_search.north<-1.56) and 
           grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0):
            grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3
    
    # Left Block detection
    if(grid_search.ds_left_value < 1535):
        
        # If direction of robot is north
        if((-0.01<grid_search.north<0.01) and 
           grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0):
            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3       
        # If direction of robot is east
        if((1.56<grid_search.north<1.58) and 
           grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0):
            grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3
        # If direction of robot is south
        if((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and 
           grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0):
            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
        # If direction of robot is west
        if((-1.58<grid_search.north<-1.56) and 
           grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0):
            grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
            

if __name__ == "__main__":
    # Call robot and define robot variable
    my_robot = Robot()
    
    grid_search.r1_curX = 0
    grid_search.r1_curY = 0
    grid_search.r1_targetX = 0
    grid_search.r1_targetY = 0
    
    grid_search.r2_curX = 0
    grid_search.r2_curY = 0
    grid_search.r2_targetX = 0
    grid_search.r2_targetY = 0
    
    grid_search.r3_curX = 0
    grid_search.r3_curY = 0
    grid_search.r3_targetX = 0
    grid_search.r3_targetY = 0
    
    cal_next_step_range.state = 0
    
    target_pos_search.found = 0
    target_pos_search.x = 0
    target_pos_search.y = 0
    target_pos_search.sng_state = 0
    target_pos_search.target_found = 0
    target_pos_search.sng_flag = 0
    
    
    
    # Start grid search proccess
    grid_search(my_robot)
    
    i = grid_search.rows - 1            
    while(i >= 0):
        print(grid_search.grid_map[i])
        i -= 1
    print("************")