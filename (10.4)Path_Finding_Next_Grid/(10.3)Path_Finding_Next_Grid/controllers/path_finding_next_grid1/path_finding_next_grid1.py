import math, collections, struct
import numpy as np
from controller import Robot, Motor, DistanceSensor, PositionSensor, Compass, InertialUnit, Emitter, Receiver


def grid_search(robot, new_x, new_y):
    grid_search.start_time = robot.getTime()
    
    grid_search.robot = robot
    
    # Define the time step 
    grid_search.time_step = 64
    
    target_pos_search.final_wait = 0 
    target_pos_search.final_wait_state = 0
    
    # Define max motor speed
    grid_search.max_speed = 6
    
    search_area.found = 0
    
    grid_search.rows = 8
    grid_search.cols = 8
    grid_search.wall = 3
    grid_search.visit = 1
    grid_search.goal = 0
    grid_search.grid_map = [[0 for i in range(grid_search.cols)] for j in range(grid_search.rows)]
    
    # draw_map(grid_search.grid_map)
    
    grid_search.next_grid_north = [[0 for i in range(grid_search.cols)]]
    grid_search.next_grid_east = [[0 for i in range(grid_search.rows)]]
    grid_search.next_grid_south = [[0 for i in range(grid_search.cols)]]
    grid_search.next_grid_west = [[0 for i in range(grid_search.rows)]]
    
    
    grid_search.cur_pos_x = new_x
    grid_search.cur_pos_y = new_y
    
    grid_search.target_temp_pos_seq = 0
        
    grid_search.increment_range = 12.464
    grid_search.movement_check = 0
    draw = 0
    grid_search.extend_check = 0
    
    block_check_call = 0
    
    grid_search.rot_angle = 0
    grid_search.rot_state = 0
    
    grid_search.tumble_state = 0
    
    grid_search.final_state = 0
    grid_search.exit_flag = 0
    
    # Define each motor
    grid_search.leftF_motor = robot.getDevice('leftF')
    grid_search.leftR_motor = robot.getDevice('leftR')
    grid_search.rightF_motor = robot.getDevice('rightF')
    grid_search.rightR_motor = robot.getDevice('rightR')
    
    # Set initial position each motor
    grid_search.leftF_motor.setPosition(0)
    grid_search.leftR_motor.setPosition(0)
    grid_search.rightF_motor.setPosition(0)
    grid_search.rightR_motor.setPosition(0)
    
    # Set initial speed 
    grid_search.leftF_motor.setVelocity(0)
    grid_search.leftR_motor.setVelocity(0)
    grid_search.rightF_motor.setVelocity(0)
    grid_search.rightR_motor.setVelocity(0)
    
    # Define distance sensor and enable
    ds_fall1 = robot.getDevice('ds_fall1')
    ds_fall1.enable(grid_search.time_step)
    
    ds_fall2 = robot.getDevice('ds_fall2')
    ds_fall2.enable(grid_search.time_step)
    
    ds_front = robot.getDevice('ds_front')
    ds_front.enable(grid_search.time_step)
    
    ds_front_r = robot.getDevice('ds_front_right')
    ds_front_r.enable(grid_search.time_step)
    
    ds_front_l = robot.getDevice('ds_front_left')
    ds_front_l.enable(grid_search.time_step)
    
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
    
    if grid_search.first_channel == 0:
        grid_search.emitter.setChannel(2)
        grid_search.receiver.setChannel(2)
        grid_search.first_channel = 1
    
    grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 1
    draw_map(grid_search.grid_map)
    
    if grid_search.emitter.getChannel() == 1:
        grid_search.emitter.setChannel(2)
        grid_search.receiver.setChannel(2)
    else:
        grid_search.emitter.setChannel(1)
        grid_search.receiver.setChannel(1)
    
    while grid_search.receiver.getQueueLength()>0:
        grid_search.receiver.nextPacket()
    
    # print((grid_search.cur_pos_x, grid_search.cur_pos_y))
    message = struct.pack("hhhh",5,1,grid_search.cur_pos_x, grid_search.cur_pos_y)
    byte = 4
    byte_message = struct.pack("h",byte)
    grid_search.emitter.send(byte_message)
    grid_search.emitter.send(message)
    
    print("new grid search")
    
    # Main loop:
    while (robot.step(grid_search.time_step) != -1 and grid_search.exit_flag != 1):
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
        grid_search.ds_front_r = ds_front_r.getValue()
        grid_search.ds_front_l = ds_front_l.getValue()
        grid_search.ds_left_value = ds_left.getValue()
        grid_search.ds_right_value = ds_right.getValue() 
        
        if(rotation_direction.counter > 0):
            rotation_direction.counter -= 1
        
        # Calculate north angle
        grid_search.north = math.atan2(compass_value[0], compass_value[2])                
        
        # print(grid_search.north)
        
        
        
        # Map update
        update_map()
        
        # draw_map(grid_search.grid_map)
                    
        receive_message()
            
        if(block_check_call == 0):
           block_check()
           block_check_call = 1
        
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
        grid_search.leftF_motor.setPosition(cal_next_step_range.rangeLeft)
        grid_search.leftR_motor.setPosition(cal_next_step_range.rangeLeft)
        grid_search.rightF_motor.setPosition(cal_next_step_range.rangeRight)
        grid_search.rightR_motor.setPosition(cal_next_step_range.rangeRight)
        
        # Set motor speeds
        grid_search.leftF_motor.setVelocity(grid_search.left_speed)
        grid_search.leftR_motor.setVelocity(grid_search.left_speed)
        grid_search.rightF_motor.setVelocity(grid_search.right_speed)
        grid_search.rightR_motor.setVelocity(grid_search.right_speed)
        
        
        
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
    if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] == 0
       or grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] == 1):
    
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 5
        
        draw_map(grid_search.grid_map)    

def receive_message():
    draw = 0
    while grid_search.receiver.getQueueLength()>0:            
        comming_message_byte = grid_search.receiver.getData()
        message_byte = struct.unpack("h",comming_message_byte)
        grid_search.receiver.nextPacket()            
        
        # print(dataList)
        
        if message_byte[0] == -1:
            
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("h",comming_message)
            
            if dataList[0] == 4: 
                search_area.r4_final = 1  
                print("r4 :",search_area.r4_final)
            if dataList[0] == 6: 
                search_area.r6_final = 1
                print("r6 :",search_area.r6_final)
            if dataList[0] == 7: 
                search_area.r7_final = 1
                print("r7 :",search_area.r7_final)
                
        elif message_byte[0] == -2:
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("hh",comming_message)
            if(dataList[0] == 0):
                grid_search.next_grid_west[0][dataList[1]] = 3
            elif(dataList[0] == 1):
                grid_search.next_grid_north[0][dataList[1]] = 3      
            elif(dataList[0] == 2):
                grid_search.next_grid_east[0][dataList[1]] = 3
            elif(dataList[0] == 3):
                grid_search.next_grid_south[0][dataList[1]] = 3
  
        else:
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("h"*message_byte[0],comming_message)
            
            lendatalist = len(dataList)
            if(grid_search.extend_check == 0):
                for j in range(1, lendatalist, 3):
                    if(dataList[j+1] > 7 or dataList[j+2] > 7):
                        # print("giriyom")
                        dataList = dataList[:j] + dataList[j+3:]
                        lendatalist -= 3
            # print(lendatalist, len(dataList))
            for i in range(1, len(dataList), 3):
                update_pos_val = dataList[i]
                update_pos_x = dataList[i+1]
                update_pos_y = dataList[i+2]
                # print(update_pos_val,update_pos_x,update_pos_y)
                if grid_search.grid_map[update_pos_x][update_pos_y] != 5:
                    if((grid_search.grid_map[update_pos_x][update_pos_y] == 4 and dataList[0] == 4)
                           or (grid_search.grid_map[update_pos_x][update_pos_y] == 6 and dataList[0] == 6)
                               or (grid_search.grid_map[update_pos_x][update_pos_y] == 7 and dataList[0] == 7)
                               or grid_search.grid_map[update_pos_x][update_pos_y] == 0
                               or grid_search.grid_map[update_pos_x][update_pos_y] == 1):
                        grid_search.grid_map[update_pos_x][update_pos_y] = update_pos_val
        
        grid_search.receiver.nextPacket()
        draw = 1
                
    if draw == 1:
        print("------mesej alindi------")
        draw_map(grid_search.grid_map)
        draw = 0

def tps_north():        
        
    # List boundary control
    if(grid_search.cur_pos_x + 1 < grid_search.rows):
        # If north direction is discoverable
        if(grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0 and target_pos_search.found == 0):
            target_pos_search.x = grid_search.cur_pos_x + 1
            target_pos_search.y = grid_search.cur_pos_y
            target_pos_search.direction = "north"
            target_pos_search.found = 1
            target_pos_search.target_found = 1
            target_pos_search.message_send = 1
            print("north")

def tps_east():
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

def tps_south():
    # List boundary control
    if(grid_search.cur_pos_x - 1 >= 0):            
        if(grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0 and target_pos_search.found == 0):
            target_pos_search.x = grid_search.cur_pos_x - 1
            target_pos_search.y = grid_search.cur_pos_y
            target_pos_search.direction = "south"
            target_pos_search.found = 1
            target_pos_search.target_found = 1
            target_pos_search.message_send = 1
    
def tps_west():
    # List boundary control
    if(grid_search.cur_pos_y - 1 >= 0):            
        if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0 and target_pos_search.found == 0):
            target_pos_search.x = grid_search.cur_pos_x
            target_pos_search.y = grid_search.cur_pos_y - 1
            target_pos_search.direction = "west"
            target_pos_search.found = 1
            target_pos_search.target_found = 1
            target_pos_search.message_send = 1

def new_coordinate():
    if(search_area.target_direcition == "north"):
        search_area.new_x = 0
        search_area.new_y = target_pos_search.y
        
    if(search_area.target_direcition == "east"):
        search_area.new_x = target_pos_search.x
        search_area.new_y = 0
        
    if(search_area.target_direcition == "south"):
        search_area.new_x = grid_search.rows - 1
        search_area.new_y = target_pos_search.y
        
    if(search_area.target_direcition == "west"):
        search_area.new_x = target_pos_search.x
        search_area.new_y = grid_search.cols - 1
 

def extend_map():
    
    grid_search.extend_check = 1
    
    if (search_area.target_direcition == "north"):
        grid_search.grid_map.extend(grid_search.next_grid_north)
        # print("next grid North")
        
    if (search_area.target_direcition == "east"):
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        grid_search.grid_map.extend(grid_search.next_grid_east)
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        
    if (search_area.target_direcition == "south"):
        grid_search.next_grid_south.extend(grid_search.grid_map)
        grid_search.grid_map = grid_search.next_grid_south
        grid_search.cur_pos_x += 1
                     
        
    if (search_area.target_direcition == "west"):
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        grid_search.next_grid_west.extend(grid_search.grid_map)
        grid_search.grid_map = grid_search.next_grid_west
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        grid_search.cur_pos_y += 1
        
    draw_map(grid_search.grid_map)
 
def target_pos_search():
# =============================================================================
# This function find the new target position on map
# =============================================================================
    receive_message()
    if (target_pos_search.found == 0 and target_pos_search.target_found == 0 and 
        target_pos_search.sng_state == 0 and grid_search.final_state == 0):
        
        if -0.01<grid_search.north<0.01:
            tps_north()        
            tps_east()        
            tps_west()         
            tps_south()
            print("north")
            
        if 1.56<grid_search.north<1.58:
            tps_east()        
            tps_north()        
            tps_south()   
            tps_west()
            print("east")
        
        if (-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):
            tps_south()   
            tps_east()        
            tps_west()
            tps_north()
            print("south")
        
        if -1.58<grid_search.north<-1.56:
            tps_west()
            tps_north()        
            tps_south()   
            tps_east()
            print("west")
        
    
    if(target_pos_search.sng_state == 0 and target_pos_search.found == 0 
       and target_pos_search.target_found == 0 and grid_search.final_state == 0):
        target_pos_search.path = search_near_grid(grid_search.grid_map, 
                                                  (grid_search.cur_pos_x, grid_search.cur_pos_y), 0)
        print(target_pos_search.path)
        if (target_pos_search.path != None):
            # Search near grid state
            target_pos_search.sng_state = 1
            target_pos_search.target_found = 1
            target_pos_search.sng_flag = 1 
            
            target_pos_search.sng_last_x, target_pos_search.sng_last_y = target_pos_search.path[-1]
            message = struct.pack("hhhh",5,1,target_pos_search.sng_last_x,target_pos_search.sng_last_y)
            byte = 4
            byte_message = struct.pack("h",byte)
            grid_search.emitter.send(byte_message)
            grid_search.emitter.send(message)
            print("sng check")
        
        else:
            grid_search.finish_time = grid_search.robot.getTime()
            print(grid_search.finish_time - grid_search.start_time)
            grid_search.final_state = 1
            
            byte_message = struct.pack("h",-1)
            grid_search.emitter.send(byte_message)
            
            message = struct.pack("h",5)
            grid_search.emitter.send(message)
            print("final state 1")
            
            target_pos_search.message_send = 1
            
            # if(search_area.target_direcition == "empty"):
            #     search_area.mission_complate = 1
            #     grid_search.exit_flag = 1
            
    if(target_pos_search.sng_state == 1):
        if(target_pos_search.sng_flag == 1):
            if(target_pos_search.path):
                target_pos_search.x, target_pos_search.y = target_pos_search.path.pop(0)  
                target_pos_search.sng_flag = 0
                target_pos_search.message_send = 1
                print("tar:",target_pos_search.x, target_pos_search.y)
            else:
                target_pos_search.sng_state = 0
                target_pos_search.target_found = 0
                
                if(grid_search.final_state == 1):
                    grid_search.exit_flag = 1
                    target_pos_search.message_send = 1 
                    
                    new_coordinate()
                    
                # print("sng_state 0")
                
        if(target_pos_search.sng_flag == 0):
            if(target_pos_search.x == grid_search.cur_pos_x and 
               target_pos_search.y == grid_search.cur_pos_y):
                target_pos_search.sng_flag = 1
            
            if(target_pos_search.x == grid_search.cur_pos_x + 1 and 
               target_pos_search.y == grid_search.cur_pos_y):
                target_pos_search.direction = "north"
            
            if(target_pos_search.x == grid_search.cur_pos_x and 
               target_pos_search.y == grid_search.cur_pos_y + 1):
                target_pos_search.direction = "east"
                
            if(target_pos_search.x == grid_search.cur_pos_x - 1 and 
               target_pos_search.y == grid_search.cur_pos_y):
                target_pos_search.direction = "south"
                # print("sng south")
                
            if(target_pos_search.x == grid_search.cur_pos_x and 
               target_pos_search.y == grid_search.cur_pos_y - 1):
                target_pos_search.direction = "west"
                
    if(target_pos_search.message_send == 1):
        # print((target_pos_search.x, target_pos_search.y))
        print("message send")
        if(grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 0
           or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 1):
            grid_search.grid_map[target_pos_search.x][target_pos_search.y] = 5
        
        if grid_search.exit_flag == 1:
            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 1
       
        byte = 4
        message = struct.pack("hhhh",5,grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y]
                              ,grid_search.cur_pos_x, grid_search.cur_pos_y)
        for j in range(len(grid_search.grid_map[0])):
            for i in range(len(grid_search.grid_map)):
                if(grid_search.grid_map[i][j] == 1 or grid_search.grid_map[i][j] == 5
                   or grid_search.grid_map[i][j] == 3):
                    message += struct.pack("hhh",grid_search.grid_map[i][j],i,j)
                    byte += 3
        
        byte_message = struct.pack("h",byte)
        grid_search.emitter.send(byte_message)
        grid_search.emitter.send(message)
        target_pos_search.message_send = 0
                
                
    if(grid_search.final_state == 1):
        if(search_area.r4_final == 1 and search_area.r6_final == 1 
           and search_area.r7_final == 1 and target_pos_search.final_wait_state == 0):
                target_pos_search.final_wait = 0
                target_pos_search.counter = 2
                target_pos_search.final_wait_state = 1
        
        if(target_pos_search.counter == 0):
            target_pos_search.final_wait = 1
        
        if target_pos_search.counter>0: target_pos_search.counter -= 1
    
    if (grid_search.final_state == 1 and target_pos_search.sng_state == 0 and target_pos_search.final_wait == 1):
        if(search_area.found == 0 and search_area.r4_final == 1
           and search_area.r6_final == 1 and search_area.r7_final == 1):            
            search_area.found = 1
            
            search_area.r4_final = 0
            search_area.r6_final = 0
            search_area.r7_final = 0
            
            # print("search_area")
            search_area.map[search_area.cur_x][search_area.cur_y] = 1
            # print(search_area.cur_x, search_area.cur_y)
            draw_map(search_area.map)
                        
            cal_next_grid()
            
            extend_map()          
            
            target_pos_search.path = search_near_grid(grid_search.grid_map, 
                                                      (grid_search.cur_pos_x, grid_search.cur_pos_y), 0)            
            # print(target_pos_search.path)
            while target_pos_search.path == [] or target_pos_search.path == None:
                print("path none")
                receive_message()
                target_pos_search.path = search_near_grid(grid_search.grid_map, 
                                                      (grid_search.cur_pos_x, grid_search.cur_pos_y), 1)
           
            # print(target_pos_search.path)
            if (target_pos_search.path != None):
                print("final state sng")
                # Search near grid state
                target_pos_search.sng_state = 1
                target_pos_search.target_found = 1
                target_pos_search.sng_flag = 1 
                
                x, y = target_pos_search.path[-1]
                message = struct.pack("hhhh",5,1,x,y)
                byte = 4
                byte_message = struct.pack("h",byte)
                grid_search.emitter.send(byte_message)
                grid_search.emitter.send(message)
                # print("sng check")
        

def search_near_grid(grid_map, start, check):
    rows = len(grid_map)
    cols = len(grid_map[0])
    
    queue = collections.deque([[start]])
    seen = set([start])
    
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        if grid_map[x][y] == grid_search.goal:
            return path
        if grid_search.final_state == 0:
            for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if (0 <= x2 < rows and 0 <= y2 < cols and 
                    (grid_map[x2][y2] == 5 or grid_map[x2][y2] == 1 or grid_map[x2][y2] == 0)
                    and (x2, y2) not in seen):
                    queue.append(path + [(x2, y2)])
                    seen.add((x2, y2))
                    
        else:
            if check == 0:
                for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                    if (0 <= x2 < rows and 0 <= y2 < cols and 
                        (grid_map[x2][y2] == 5 or grid_map[x2][y2] == 1 or grid_map[x2][y2] == 0)
                        and (x2, y2) not in seen):
                        queue.append(path + [(x2, y2)])
                        seen.add((x2, y2))

            else:                      
                for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                    if 0 <= x2 < rows and 0 <= y2 < cols and grid_map[x2][y2] != grid_search.wall and (x2, y2) not in seen:
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
                # if(cal_next_step_range.range_fix == 0):
                cal_next_step_range.rangeLeft = grid_search.ps_left_value + grid_search.increment_range
                cal_next_step_range.rangeRight = grid_search.ps_right_value + grid_search.increment_range
                # else:
                #     cal_next_step_range.rangeLeft = grid_search.ps_left_value + grid_search.increment_range + 0.8866
                #     cal_next_step_range.rangeRight = grid_search.ps_right_value + grid_search.increment_range + 0.8866
                #     cal_next_step_range.range_fix = 0
                #     print("fixed range")
                    
                cal_next_step_range.state = 1
                grid_search.movement_check = 1
                print("Calculate range for next step")
                
        # Not same and if target direction is north then turn north
        elif(target_pos_search.direction == "north"):
            grid_search.rot_angle = 0
            grid_search.rot_state = 1
            cal_next_step_range.state = 1
            cal_next_step_range.range_fix = 1
            
        # Not same and if target direction is north then turn east
        elif(target_pos_search.direction == "east"):
            grid_search.rot_angle = 1
            grid_search.rot_state = 1
            cal_next_step_range.state = 1
            cal_next_step_range.range_fix = 1
        
        # Not same and if target direction is north then turn south
        elif(target_pos_search.direction == "south"):
            grid_search.rot_angle = 2
            grid_search.rot_state = 1
            cal_next_step_range.state = 1
            cal_next_step_range.range_fix = 1
            print("turning south")
        
        # Not same and if target direction is north then turn west
        elif(target_pos_search.direction == "west"):
            grid_search.rot_angle = 3
            grid_search.rot_state = 1
            cal_next_step_range.state = 1
            cal_next_step_range.range_fix = 1

def rotation_direction(angle):
# =============================================================================
# This function detect rotate direction according to the direction of the return
# =============================================================================

    # Direction of return is north
    if(angle == 0):
        if((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            
        elif(1.56<grid_search.north<1.58 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            
        elif(-1.58<grid_search.north<-1.56 and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
    
    # Direction of return is east
    if(angle == 1):
        # print(rotation_direction.counter)
        if(-1.58<grid_search.north<-1.56 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            # print("if")
            
        elif(-0.01<grid_search.north<0.01 and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            # print("elif")
            
        elif((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            
    
    # Direction of return is south     
    if(angle == 2):
        
        if(-0.01<grid_search.north<0.01 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            
        elif(1.56<grid_search.north<1.58 and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.slow = 45
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.check = 1
            
        elif(-1.58<grid_search.north<-1.56 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.slow = 45
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.check = 1
    
    # Direction of return is west
    if(angle == 3):
        
        if(1.56<grid_search.north<1.58 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            
        elif(-0.01<grid_search.north<0.01 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            
        elif((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
    
    if(rotation_direction.counter < rotation_direction.counter_start - rotation_direction.slow and 
       rotation_direction.slow_check == 0):
        grid_search.left_speed /= 48
        grid_search.right_speed /= 48       
        rotation_direction.slow_check = 1
        # print("slowing")
    
    if rotation_direction.counter == 0 and rotation_direction.slow_check2 == 0:
        grid_search.left_speed /= 10
        grid_search.right_speed /= 10
        rotation_direction.slow_check2 = 1
    
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
            if(-0.00009<grid_search.north<0.0009):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                
                
        
        # If rotate angle is 1
        if(grid_search.rot_angle == 1):
            
            # Rotation direction according to target rotation direction
            rotation_direction(1)
            
            # Robot then turn east
            if(1.57075<grid_search.north<1.57085):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                # print("turn end")
        
        # If rotate angle is 2
        if(grid_search.rot_angle == 2):
            
            # Rotation direction according to target rotation direction
            rotation_direction(2)
            
            # Robot then turn south
            if((-3.14168<grid_search.north<-3.14152 or 3.14152<grid_search.north<3.14161)):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                
        
        # If rotate angle is 3
        if(grid_search.rot_angle == 3):
            
            # Rotation direction according to target rotation direction
            rotation_direction(3)
            
            # Robot then turn west
            if(-1.57085<grid_search.north<-1.57075):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                
    # Else go forward
    else:
        if(grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 4
           or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 6
           or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 7
           or grid_search.ds_front_value < 250 
           or grid_search.ds_front_l < 260 
           or grid_search.ds_front_r < 260):
            
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                # print("waiting")
                # print(rotate_check.counter)
                if rotate_check.counter == -1:   rotate_check.counter = 400
                if rotate_check.counter > 0:    rotate_check.counter -= 1
                if (rotate_check.counter == 0):
                    target_pos_search.sng_state = 0
                    target_pos_search.target_found = 0
                    cal_next_step_range.state = 0
                    target_pos_search.direction = "empty"
                    if(grid_search.grid_map[target_pos_search.sng_last_x][target_pos_search.sng_last_y] == 1):
                        grid_search.grid_map[target_pos_search.sng_last_x][target_pos_search.sng_last_y] = 0
                        # print(target_pos_search.sng_last_x, target_pos_search.sng_last_y)
                        
                    rotate_check.counter = -1
                    print("It's tooooo much time")
            
        else: 
            if(target_pos_search.x == grid_search.cur_pos_x 
                and target_pos_search.y == grid_search.cur_pos_y):
                pass
            else:
                grid_search.left_speed = grid_search.max_speed
                grid_search.right_speed = grid_search.max_speed
                # print("forward")
            

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
        rotate_check.counter = -1
        
        print("reset")
        if(target_pos_search.sng_state == 1):
            if(target_pos_search.sng_flag == 0):
                target_pos_search.sng_flag = 1
        else:
            target_pos_search.target_found = 0
        # print("rotate pos check")
        
        # print("cur", grid_search.cur_pos_x, grid_search.cur_pos_y)
        # print("tar", target_pos_search.x, target_pos_search.y)
        
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 1
        
        # Now robot current position is target postion
        grid_search.cur_pos_x = target_pos_search.x
        grid_search.cur_pos_y = target_pos_search.y
        
        draw_map(grid_search.grid_map)
        # print(grid_search.cur_pos_x)
        # print(grid_search.cur_pos_y)
        
        # Check front, left or right sight of robot for any block
        
        block_check()
        
def block_check():
    
    # Front Block detection
    if(grid_search.ds_front_value < 1491 or grid_search.ds_front_l < 1542 or grid_search.ds_front_r < 1542):
        
        # If direction of robot is north
        if(-0.01<grid_search.north<0.01):
            if(grid_search.cur_pos_x + 1 < grid_search.rows):
                if grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3   
                
            elif(grid_search.next_grid_north[0][grid_search.cur_pos_y] == 0):
                
                grid_search.next_grid_north[0][grid_search.cur_pos_y] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",1,grid_search.cur_pos_y)
                grid_search.emitter.send(message)
        
        # If direction of robot is east
        if(1.56<grid_search.north<1.58):
            if(grid_search.cur_pos_y + 1 < grid_search.cols):
                if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
                
            elif grid_search.next_grid_east[0][grid_search.cur_pos_x] == 0:
                
                grid_search.next_grid_east[0][grid_search.cur_pos_x] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",2,grid_search.cur_pos_x)
                grid_search.emitter.send(message)
                
        # If direction of robot is south
        if(-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):
            if(grid_search.cur_pos_x - 1 >= 0):
                if grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
            
            elif grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0:
                grid_search.next_grid_south[0][grid_search.cur_pos_y] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",3,grid_search.cur_pos_y)
                grid_search.emitter.send(message)
        
        # If direction of robot is west
        if(-1.58<grid_search.north<-1.56):
            if(grid_search.cur_pos_y - 1 >= 0):
                if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
            
            elif grid_search.next_grid_south[0][grid_search.cur_pos_x] == 0:
                grid_search.next_grid_west[0][grid_search.cur_pos_x] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",0,grid_search.cur_pos_x)
                grid_search.emitter.send(message)
                
    # Right Block detection
    if(grid_search.ds_right_value < 1675):
        
        # If direction of robot is north
        if(-0.01<grid_search.north<0.01):
            if(grid_search.cur_pos_y + 1 < grid_search.rows):
                if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
            elif grid_search.next_grid_south[0][grid_search.cur_pos_x] == 0:
                
                grid_search.next_grid_east[0][grid_search.cur_pos_x] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",2,grid_search.cur_pos_x)
                grid_search.emitter.send(message)
        
        # If direction of robot is east
        if(1.56<grid_search.north<1.58):
            if(grid_search.cur_pos_x - 1 >= 0):
                if grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
            
            elif grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0:
                grid_search.next_grid_south[0][grid_search.cur_pos_y] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",3,grid_search.cur_pos_y)
                grid_search.emitter.send(message)
        
        # If direction of robot is south
        if(-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):
            if(grid_search.cur_pos_y - 1 >= 0):
                if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
            
            elif grid_search.next_grid_south[0][grid_search.cur_pos_x] == 0:
                grid_search.next_grid_west[0][grid_search.cur_pos_x] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",0,grid_search.cur_pos_x)
                grid_search.emitter.send(message)
        
        # If direction of robot is west
        if(-1.58<grid_search.north<-1.56): 
            if(grid_search.cur_pos_x + 1 < grid_search.rows):
                if grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3     
                
            elif grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0:
                grid_search.next_grid_north[0][grid_search.cur_pos_y] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",1,grid_search.cur_pos_y)
                grid_search.emitter.send(message)
                
    
    # Left Block detection
    if(grid_search.ds_left_value < 1675):
        
        # If direction of robot is north
        if(-0.01<grid_search.north<0.01):
            if(grid_search.cur_pos_y - 1 >= 0):
                if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
            
            elif grid_search.next_grid_south[0][grid_search.cur_pos_x] == 0:
                grid_search.next_grid_west[0][grid_search.cur_pos_x] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",0,grid_search.cur_pos_x)
                grid_search.emitter.send(message)
        
        # If direction of robot is east
        if(1.56<grid_search.north<1.58):
            if(grid_search.cur_pos_x + 1 < grid_search.rows):
                if grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3     
                
            elif grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0:
                grid_search.next_grid_north[0][grid_search.cur_pos_y] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",1,grid_search.cur_pos_y)
                grid_search.emitter.send(message)
        
        # If direction of robot is south
        if(-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15): 
            if(grid_search.cur_pos_y + 1 < grid_search.rows):
                if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
            elif grid_search.next_grid_south[0][grid_search.cur_pos_x] == 0:
                grid_search.next_grid_east[0][grid_search.cur_pos_x] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",2,grid_search.cur_pos_x)
                grid_search.emitter.send(message)
        
        # If direction of robot is west
        if(-1.58<grid_search.north<-1.56):
            if(grid_search.cur_pos_x - 1 >= 0):
                if grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0:
                    grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
            
            elif grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0:
                grid_search.next_grid_south[0][grid_search.cur_pos_y] = 3
                
                byte_message = struct.pack("h",-2)
                grid_search.emitter.send(byte_message)
                message = struct.pack("hh",3,grid_search.cur_pos_y)
                grid_search.emitter.send(message)
                

def cal_next_grid():
    if(cal_next_grid.found == 0):
        # print("calculate next grid")
        if(0 <= search_area.cur_x + 1 < search_area.rows):
            if(search_area.map[search_area.cur_x + 1][search_area.cur_y] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x + 1
                search_area.target_y = search_area.cur_y
                search_area.target_direcition = "north"
                cal_next_grid.found = 1
                # print("north")
            
        if(0 <= search_area.cur_y + 1 < search_area.cols):  
            if(search_area.map[search_area.cur_x][search_area.cur_y + 1] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x
                search_area.target_y = search_area.cur_y + 1
                search_area.target_direcition = "east"
                cal_next_grid.found = 1
                # print("east")
        
        if(0 <= search_area.cur_x - 1):
            if(search_area.map[search_area.cur_x - 1][search_area.cur_y] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x - 1
                search_area.target_y = search_area.cur_y
                search_area.target_direcition = "south"
                cal_next_grid.found = 1
                # print("south")
        
        if(0 <= search_area.cur_y - 1):
            if(search_area.map[search_area.cur_x][search_area.cur_y - 1] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x
                search_area.target_y = search_area.cur_y - 1
                search_area.target_direcition = "west"
                cal_next_grid.found = 1 
                # print("west")


def search_area(robot):
    search_area.mission_complate = 0
    
    search_area.cols = 5
    search_area.rows = 5
    
    search_area.map = [[0 for i in range(search_area.cols)] for j in range(search_area.rows)]
    
    search_area.cur_x = 0
    search_area.cur_y = 0
    
    search_area.target_x = 0
    search_area.target_y = 0
    
    search_area.new_x = 0
    search_area.new_y = 3
    
    search_area.start_time = robot.getTime()
    
    while search_area.mission_complate != 1:
        search_area.target_direcition = "empty"
        
        cal_next_grid.found = 0
        
        # print(search_area.cur_x, search_area.cur_y)
        
        grid_search(robot, search_area.new_x, search_area.new_y)
        
        
        # if(search_area.cur_x == search_area.target_x and 
        #    search_area.cur_y == search_area.target_y):
        #     search_area.mission_complate = 1
        
        search_area.cur_x = search_area.target_x
        search_area.cur_y = search_area.target_y
                    
    search_area.finish_time = robot.getTime()    
    
    print(search_area.finish_time - search_area.start_time)

if __name__ == "__main__":
    # Call robot and define robot variable
    my_robot = Robot()
    
    grid_search.const_counter = 65
    grid_search.first_channel = 0
    
    cal_next_step_range.state = 0
    cal_next_step_range.range_fix = 0
    
    target_pos_search.found = 0
    target_pos_search.x = 0
    target_pos_search.y = 0
    target_pos_search.sng_last_x = 0
    target_pos_search.sng_last_y = 0
    target_pos_search.sng_state = 0
    target_pos_search.target_found = 0
    target_pos_search.sng_flag = 0
    target_pos_search.path = []
    target_pos_search.counter = 0
    
    cal_next_grid.found = 0
        
    search_area.found = 0    
    search_area.r4_final = 0
    search_area.r6_final = 0
    search_area.r7_final = 0
    
    rotate_check.counter = -1
    
    rotation_direction.check = 0
    rotation_direction.speed_set = 0
    rotation_direction.counter = -1
    rotation_direction.slow = 0
    rotation_direction.slow_check = 0
    rotation_direction.slow_check2 = 0
    rotation_direction.counter_start = 0
    
    # Start grid search proccess
    search_area(my_robot)
    
    