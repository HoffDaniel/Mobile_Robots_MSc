function sysCall_init()--###############################################################
    --[[
    This is the fist function called
    We use it to initiate variables and other
    ]]
-- ---------------------------------------------------------
--                   OBJECT HANDLEs stuff
-- --------------------------------------------------------- 
    usensors={-1,-1,-1,-1,-1,-1,-1,-1}
    n_sensors = 8
    for i=1,n_sensors,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    
    beacons={}
    n_beacons = 1 --IF we have more than 1 beacon
    for i=1,n_beacons,1 do
        beacons[i]=sim.getObjectHandle("beacon"..i)
    end

    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")    
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    pioneer = sim.getObjectHandle("Pioneer_p3dx")
    
    
    

-- ---------------------------------------------------------
--                   CONSTANTs stuff
-- --------------------------------------------------------- 
    --SENSORS
    detect={}
    for i=1,n_sensors,1 do
        detect[i] = 10
    end
    sonarAngles ={90,50,30,10,-10,-30,-50,-90}
    --BEACONS
    beacon_threshold = 7 --CHANGE THIS TO modify the radius from which the beacon is detectable to the robot
    visit_beacon = 1 --used when more beacons 
    detect_beacon={} --holds the distance(s) to the beacon
    beacon_Pos={}
    for i=1,n_beacons,1 do
        detect_beacon[i] = -1
        beacon_Pos[i] = {x = 99, y = 99}
    end
    
    --DEFAULT
    v0=2 --Reference/default speed
    setDist = 0.5
    robot_radius = 0.2
    
-- ---------------------------------------------------------
--                   TASKS stuff
-- ---------------------------------------------------------
    task = {}
    task[1] = false --FIND ROOM CENTRE
    task[2] = false --FIND BACON
    task[3] = false --FINISH MAPPING
    task[4] = false --RETURN to ROOM CENTRE
    
-- ---------------------------------------------------------
--                   GRID stuff
-- --------------------------------------------------------- 

  
    floor_size = 15 --current size of the "environment" /floor can be increased for a bigger environment
    steps = 0.5 --distance to next point in grid (if changing multiply or divide by 2(remember to change size_point as well for nicer visuals)
    check_range = 3
    grid_dimension = 1 + floor_size/steps --+1 cause lua starts at 1
    grid_shift = floor_size/2 --
    
    --grid={}
    occupancy_grid={}

    
    size_point = 3.4 *2*2-- change the size of the points drawn to the scene
    drawing_Grid_Unknown = sim.addDrawingObject(sim.drawing_painttag, size_point, steps/2, -1, 42000,{0.5 , 0.5, 0.5}) -- 3.4
    
    drawing_Grid_Free = sim.addDrawingObject(sim.drawing_painttag, size_point, steps/2, -1, 42000,{0 , 1, 0})
    drawing_Grid_Fr = sim.addDrawingObject(sim.drawing_painttag, size_point, steps/2, -1, 42000,{0 , 0.5, 0})
    
    drawing_Grid_Occupied = sim.addDrawingObject(sim.drawing_painttag,size_point, steps/2, -1, 42000,{1 , 0, 0})
    drawing_Grid_Occ = sim.addDrawingObject(sim.drawing_painttag,size_point, steps/2, -1, 42000,{0.5 , 0, 0})
    --sim.drawing_points
    
    
    
    for i=1, grid_dimension do
        occupancy_grid[i] = {}
        for j=1, grid_dimension do
        x = i-1
        y = j-1
        
        occupancy_grid[i][j] = {
                x = (x*steps) - grid_shift ,
                y = (y*steps) - grid_shift, 
                occupied = 0.5,
                neigh = {{i-1, j-1},{i-1, j},{i-1, j+1},{i, j-1},{i, j+1},{i+1, j-1},{i+1, j},{i+1, j+1}},
                f = 0,
                g = 0,
                h = 0,
                parent = 0
                }
        grid_point = {occupancy_grid[i][j].x, occupancy_grid[i][j].y, 15}--occupancy_grid[i][j].occupied}
        sim.addDrawingObjectItem(drawing_Grid_Unknown, grid_point)        
                 
        end
    end
    grid_size = grid_dimension * grid_dimension -- n * m
    
-- ---------------------------------------------------------
--                   PID stuff
-- --------------------------------------------------------- 
    I_max = 10
    I_errors = {}
    for i=1, I_max do
        I_errors[i] = 0
    end
    I_count = 1
    I_Average = 0
    
    error = 0
    previous_error = 0
    
    p_gain = 99--82--66--99----100 --
    i_gain = 0.42--0.1--0.1----1--
    d_gain = 66--42--99--132----42--
    
    RMSE = 0 -- root-mean-square-error => the smaller this value the better the PID
    
    
-- ---------------------------------------------------------
--                   FSM stuff
-- --------------------------------------------------------- 
    State = {"Search_Room","Random_Wander","Wander_Beacon", "Follow_Wall", "Return_Centre", "IDLE"}
    boolState = {Wander = true, Follow = true, Idle = true}
    robot_state = State[1] --Robots starting state
    time_state_entered = 0 --Stores the time a state is entered 
    time_in_state = 0  

-- ---------------------------------------------------------
--                   Search* stuff
-- ---------------------------------------------------------  
    search_stage = 0
    search_corner = {c1 = {x=0,y=0, isCorner = false},
                     c2 = {x=0,y=0, isCorner = false},
                     c3 = {x=0,y=0, isCorner = false},
                     c4 = {x=0,y=0, isCorner = false}
                     }
    corner_time = 0
    Room_centre = {x = 0, y = 0, isVisited = false}
    Door = {}
    ---------------------
    -- A Star
    ---------------------
    openSet = {}
    closedSet = {}
    current = {}
    aPath = {} --stores a found path
    
    drawing_Goal = sim.addDrawingObject(sim.drawing_painttag, size_point, 0.0, -1, 42000,{0.3 , 0.3, 1})
    drawing_Current = sim.addDrawingObject(sim.drawing_painttag, size_point, 0.0, -1, 42000,{0.7 , 0.0, 0.5})
    drawing_Path = sim.addDrawingObject(sim.drawing_painttag, size_point, 0.0, -1, 42000,{0.3,0.7,0.9})
    drawing_Open = sim.addDrawingObject(sim.drawing_painttag, size_point, 0.0, -1, 42000,{0.8 , 0.8, 0})
    drawing_Closed = sim.addDrawingObject(sim.drawing_painttag, size_point, 0.0, -1, 42000,{1 , 0.6, 0})
    
-- ---------------------------------------------------------
--                   Wander stuff
-- ---------------------------------------------------------     
    random_wander = 0 --math.random(1,10)
    randX = math.floor(math.random(1,grid_dimension))
    randY = math.floor(math.random(1,grid_dimension))

    
-- ---------------------------------------------------------
--              Sense/Mapping/Data/Info stuff
-- ---------------------------------------------------------
    start = true    
    starting_pos = sim.getObjectPosition(pioneer,-1)
    
    
    
    orientation = sim.getObjectOrientation(pioneer,-1)
    robot_heading = math.deg(orientation[3]) 
    
    
    
    drawing_RobPos = sim.addDrawingObject(sim.drawing_painttag, size_point, 0.0, -1, 42000,{0 , 0, 1})
    drawing_SpecialPoints = sim.addDrawingObject(sim.drawing_painttag, size_point, 0.0, -1, 42000,{1 , 0, 1})
    
    --A Star
    
    
    
    
    --[[
    drawing_Lines = sim.addDrawingObject(sim.drawing_lines, 3, 0.0, -1, 42000)
    drawing_Rect = sim.addDrawingObject(sim.drawing_quadpoints, 3, 0.0, -1, 42000)
    drawing_Sphere = sim.addDrawingObject(sim.drawing_spherepoints, 0.5, 0.0, -1, 42000)
    --]]
  
   
    --External DATA stuff
    --1107 x 623
    --PC
    
    
 end
 
function sysCall_cleanup()--###############################################################
 --[[
 Clean up files and other
 ]]
    --io.close(file)
 
end 

function sysCall_sensing()--###############################################################
    
    --[[
        We could use this to separate what our system senses from other activities
        
        sysCall_sensing() is executed before sysCall_actuation()
    ]]
   
end

function sysCall_actuation() 
    --clear any previous values for the velocity ->
    vLeft = v0 
    vRight = v0

-- ---------------------------------------------------------
--                    Sense/Mapping/Data/Info/...
-- --------------------------------------------------------- 
    --ROBOT
    update_Robot_Info()
    
    --SENSORS    
    update_Sensor_Info()
    
    --GRID
    --update_Drawing_Grid() maybe do ever X time
    
    --BEACON(s)
    update_Beacon_Info()

    
    --File
    --Print out some info
    print("[robot_state: " .. robot_state .. " ]")
    print("[time_in_state: " .. getDeci(time_in_state,3) .. " ]")
    
    
-- ###############################################################################################################################
-- #                                                            ################################################################
-- #                    FINITE STATE MACHINE                     ################################################################
-- #                                                              ################################################################
-- ****************************************************************################################################################    
    
-- ---------------------------------------------------------
--                    STATE wander (random)
-- ---------------------------------------------------------    
    if (robot_state == "Random_Wander")then
        time_in_state = sim.getSimulationTime() - time_state_entered
        
        
        --In this state do:
        
        if(check_L()) then
            turnRight()
        elseif(check_R())then
            turnLeft()
        elseif(check_MapComp(grid_size, 20))then
            task[3] = true        
        else
            --vLeft = v0
            --vRight = v0
            wander_Rand(3)
            --wander_rand_PointCloud(time_in_state)
        end
        
        --vLeft = 0
        --vRight = 0
        --Conditions:
        if(time_in_state > 1000) then 
            robot_state = "IDLE" 
            time_state_entered = sim.getSimulationTime()
        elseif (detect_beacon[visit_beacon] ~= 99 and task[2] == false) then 
            robot_state = "Wander_Beacon" 
            time_state_entered = sim.getSimulationTime()
        elseif (task[3] and task[2]) then 
            robot_state = "Return_Centre" 
            time_state_entered = sim.getSimulationTime()
        else robot_state = "Random_Wander" end
        
-- ---------------------------------------------------------
--                    STATE wander (to bacon(s))
-- ---------------------------------------------------------    
    elseif (robot_state == "Wander_Beacon")then
        time_in_state = sim.getSimulationTime() - time_state_entered
        print("Visit beacon: " .. visit_beacon)
        --In this state do:
        if(check_L(0.3)) then
            turnRight()
        elseif (check_R(0.3))then
            turnLeft()
        elseif (detect_beacon[visit_beacon] ~= 99) then 
            
            b_pos = {x = beacon_Pos[visit_beacon].x , y = beacon_Pos[visit_beacon].y }
            --move_to_point(b_pos.x,b_pos.y)
            sim.addDrawingObjectItem(drawing_SpecialPoints, {b_pos.x , b_pos.y, 17})
            if(detect_beacon[visit_beacon]< setDist)then
                visit_beacon = visit_beacon + 1
            end
            
            rob_pos = {x = robot_pos[1], y = robot_pos[2]}
            grid_po = get_PointIn_Grid(rob_pos)
            Start = occupancy_grid[grid_po.x][grid_po.y]
            
            grid_be = get_PointIn_Grid(b_pos)
            Goal = occupancy_grid[grid_be.x][grid_be.y]
            occupancy_grid[grid_be.x][grid_be.y].occupied = 0.5 --maybe other value //special for beacons?
            
            if(table.getn(aPath) > 0) then
                --follow_Path/Points
                --vLeft=1
                --vRight=1
                follow_path(aPath)
                if((math.floor(time_in_state) % 10) == 0)then
                    reset_aStar()
                end
                
            else
                print(aStar(Start,Goal))
                vLeft=1
                vRight=1
            end    
        end
        if(visit_beacon > n_beacons) then 
            task[2] = true
            reset_aStar() -- reset path value for next A* search
            
        end
        
        --Conditions:
        if(time_in_state > 1000) then 
            robot_state = "IDLE" 
            time_state_entered = sim.getSimulationTime()
        elseif(task[2]) then            
            robot_state = "Random_Wander" 
            time_state_entered = sim.getSimulationTime()
        else robot_state = "Wander_Beacon" end
-- ---------------------------------------------------------
--                    STATE Search_Room ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ---------------------------------------------------------    
    elseif (robot_state == "Search_Room")then
        time_in_state = sim.getSimulationTime() - time_state_entered
        --In this state do:
        search_Room_Centre()
        
        --Conditions:
        if(time_in_state > 500) then 
            robot_state = "IDLE" 
            time_state_entered = sim.getSimulationTime()
        elseif(task[1]) then 
            robot_state = "Random_Wander" 
            time_state_entered = sim.getSimulationTime()
        else robot_state = "Search_Room" end
-- ---------------------------------------------------------
--                    STATE Return_Centre ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ---------------------------------------------------------    
    elseif (robot_state == "Return_Centre")then
        time_in_state = sim.getSimulationTime() - time_state_entered
        --In this state do:
        rob_pos = {x = robot_pos[1], y = robot_pos[2]}
        grid_po = get_PointIn_Grid(rob_pos)
        Start = occupancy_grid[grid_po.x][grid_po.y]
        
        grid_RoCe = get_PointIn_Grid(Room_centre)
        Goal = occupancy_grid[grid_RoCe.x][grid_RoCe.y]
        --In this state do:
        if(check_L(0.2)) then
            turnRight()
        elseif (check_R(0.2))then
            turnLeft()
        elseif(table.getn(aPath) > 0) then
            res = follow_path(aPath)
            if(res)then
                task[4] = true
                reset_aStar()
            elseif((math.floor(time_in_state) % 15) == 0)then
                reset_aStar()
            end
        else
            print(aStar(Start,Goal))
            vLeft=0
            vRight=0
        end    
        
        if(check_inCentre(Room_centre, rob_pos, 0.5))then
            task[4] = true
        end
        --Conditions:
        if(time_in_state > 100 and task[4]) then 
            print("THIS IS THE END")
            print("[Completed Tasks: " , task)
            aStar(Start,Goal, true)
        elseif(time_in_state > 500) then 
            robot_state = "IDLE" 
            time_state_entered = sim.getSimulationTime()
        else robot_state = "Return_Centre" end
                
-- ---------------------------------------------------------
--                    STATE Idle
-- ---------------------------------------------------------
    elseif (robot_state == "IDLE" )then
        --In this state do:
        vLeft = 0
        vRight = 0
        print("Help I'm stuck or something")
         --Conditions:
        
    end                   



-- ****************************************************************###############################################################
-- #                                                              ################################################################
-- #                                                             ################################################################
-- #                                                           ################################################################
-- ###############################################################################################################################


-- ---------------------------------------------------------
--                    Act
-- ---------------------------------------------------------    
    --Set the velocity of the actuators   
    setRobotVelo(vLeft,vRight)        
end 

-- ---------------------------------------------------------###############################################################

-- -------------------- MY FUNCTIONS -----------------------###############################################################

-- ---------------------------------------------------------###############################################################

function setRobotVelo(vL, vR)
    sim.setJointTargetVelocity(motorLeft,vL)
    sim.setJointTargetVelocity(motorRight,vR)
end

function update_Robot_Info()
    robot_pos = sim.getObjectPosition(pioneer,-1)
    orientation = sim.getObjectOrientation(pioneer,-1)
    robot_heading = math.deg(orientation[3])
    

    --rob_point = {x = robot_pos[1], y = robot_pos[2]}
    --grid_key = get_PosIn_Grid(rob_point)
    
   -- rob_point = {
    --    occupancy_grid[grid_key[1].x][grid_key[1].y].x,
    --    occupancy_grid[grid_key[1].x][grid_key[1].y].y,
    --    15.5}
    
    rob_point = {robot_pos[1], robot_pos[2], 18.5}
    
    sim.addDrawingObjectItem(drawing_RobPos, nil)
    sim.addDrawingObjectItem(drawing_RobPos, rob_point)
        --update GRID
        posX = math.floor((robot_pos[1])/steps + grid_shift/steps ) +1
        posY = math.floor((robot_pos[2])/steps + grid_shift/steps ) +1
        
        --posX = (math.floor(robot_pos[1]))/steps + grid_shift*4 
        --posY = (math.floor(robot_pos[2]))/steps + grid_shift*4 
        
        if(posX < 1)then posX = 1 
        elseif (posX > grid_dimension)then  posX = grid_dimension 
        end
        
        if(posY < 1)then posY = 1 
        elseif (posY > grid_dimension)then  posY = grid_dimension
        end       
        
        
        for i=1, check_range do
            for j=1, check_range do
                iX = posX - 1 + i
                jY = posY - 1 + j
        
                range = steps/2
                range_X = {x1 = occupancy_grid[iX][jY].x -range ,x2 = occupancy_grid[iX][jY].x +range}
                range_Y = {y1 = occupancy_grid[iX][jY].y -range ,y2 = occupancy_grid[iX][jY].y +range}
                if(robot_pos[1] < range_X.x2 and
                    robot_pos[1] > range_X.x1)then
                    if(robot_pos[2] < range_Y.y2 and
                        robot_pos[2] > range_Y.y1)then
                        occupancy_grid[iX][jY].occupied = 0
                        update_Drawing_Grid_Point(iX,jY)
                    end
                end
            end
        end
end
function update_Beacon_Info()
    for i=1,n_beacons,1 do 
        res,dist=sim.checkDistance(pioneer,beacons[i],beacon_threshold)
        if(res > 0) then             
            detect_beacon[i] = dist[7]
            beacon_Pos[i].x = dist[4]--x = dist[4]
            beacon_Pos[i].y = dist[5]--y = dist[5]
        else 
            detect_beacon[i] = 99
        end        
    end
end

function update_Sensor_Info()
    sum = 0
    for i=1,n_sensors,1 do 
        res,dist,point,hand,norm=sim.readProximitySensor(usensors[i])
        if(res > 0) then ------------------------------------------------------------------------------------------
            detect[i] = dist
            
            occ = detect_GlobalPos(detect[i],sonarAngles[i])
            
            occX = math.floor(occ[1]/steps + grid_shift/steps)+1
            occY = math.floor(occ[2]/steps + grid_shift/steps)+1
            
            
            
            
            
            if(occX < 1)then occX = 1 
            elseif (occX > grid_dimension)then  occX = grid_dimension 
            end
            
            if(occY < 1)then occY = 1 
            elseif (occY > grid_dimension)then  occY = grid_dimension
            end
            
            for i=1, check_range do
                for j=1, check_range do
                    iX = occX - 1 + i
                    jY = occY - 1 + j
                    
                    if(iX < 1)then iX = 1 
                    elseif (iX > grid_dimension)then  iX = grid_dimension 
                    end
                    
                    if(jY < 1)then jY = 1 
                    elseif (jY > grid_dimension)then  jY = grid_dimension
                    end
                    
                    range = steps/2 --/2
                    range_X = {x1 = occupancy_grid[iX][jY].x -range ,x2 = occupancy_grid[iX][jY].x +range}
                    range_Y = {y1 = occupancy_grid[iX][jY].y -range ,y2 = occupancy_grid[iX][jY].y +range}
                    if(occ[1] < range_X.x2 and
                        occ[1] > range_X.x1)then
                        if(occ[2] < range_Y.y2 and
                            occ[2] > range_Y.y1)then
                            if(dist < 1)then
                                if(occupancy_grid[iX][jY].occupied == 1 or 
                                   occupancy_grid[iX][jY].occupied == 0 )then
                                else
                                    occupancy_grid[iX][jY].occupied = 1
                                    update_Drawing_Grid_Point(iX,jY)
                                end
                            else
                                if(occupancy_grid[iX][jY].occupied == 1 or 
                                   occupancy_grid[iX][jY].occupied == 0 )then
                                else
                                    occupancy_grid[iX][jY].occupied = 0.75
                                    update_Drawing_Grid_Point(iX,jY)
                                end
                            end
                        end
                    end
                end
            end
            
            Dist = math.floor(dist)-1
           
            for a=1, Dist do
                occ = detect_GlobalPos(a,sonarAngles[i])
                detect_point = {x = occ[1], y = occ[2]}
                
                check_PointIn_Grid(detect_point, 0.25 , steps/2)
                
            end--]]
        
            --sim.addDrawingObjectItem(drawing_RobPos, occ) 
         
        else -------------------------------------------------------------------------------------------------------------
            detect[i] = 10 
            
            sensor_range = 4
            check = sensor_range -1
            
            for a=1, check do
                occ = detect_GlobalPos(a,sonarAngles[i])
                detect_point = {x = occ[1], y = occ[2]}    
                
                check_PointIn_Grid(detect_point, 0.25 , steps/2)
            end
        end
        sum = sum + detect[i]
    end
end
function wander_Rand(t)
        if(random_wander == 0) then
            time_wander = sim.getSimulationTime()
    
            random_wander = math.random(1,5)
            random_turn = math.random(1,5)
            
            
        else
            time_spent = sim.getSimulationTime() - time_wander
            if(time_spent < t)then
                vLeft = random_wander
                vRight = random_wander 
            elseif(time_spent >= t and time_spent < (t+2))then
                vLeft = random_wander
                vRight = random_turn
            else
                random_wander = 0
                
            end
            
        end    
end

function wander_rand_PointCloud(Time)
    --print(math.floor(Time) % 10)
    if((math.floor(Time) % 25) == 0)then
        randX = math.floor(math.random(1,grid_dimension))
        randY = math.floor(math.random(1,grid_dimension))
        --aPath = {}
    end
    
    if(occupancy_grid[randX][randY].occupied ~= 0.5) then
        randX = math.floor(math.random(1,grid_dimension))
        randY = math.floor(math.random(1,grid_dimension))
        --aPath = {}
    end
      
    if(occupancy_grid[randX][randY].occupied == 0.5) then
        grid_point = {occupancy_grid[randX][randY].x ,occupancy_grid[randX][randY].y,16}
        print("Going To Point: " .. grid_point[1] .. ", " .. grid_point[2])
        move_to_point(grid_point[1], grid_point[2])
        
        
        
        --[[ A*
        rob_pos = {x = robot_pos[1], y = robot_pos[2]}
        grid_po = get_PointIn_Grid(rob_pos)
        Start = occupancy_grid[grid_po.x][grid_po.y]
        Goal = occupancy_grid[randX][randY]
        if(table.getn(aPath) > 0) then
            --follow_Path/Points
            vLeft=0
            vRight=0
        else
            aStar(Start,Goal)
        end  
        --]]
    end
end
function moveNOW(vL,vR) --like setRobotVelo
    vLeft = vL
    vRight = vR    
    setRobotVelo(vLeft,vRight)
end
function turnRight()
    vLeft=2
    vRight=-2
    setRobotVelo(vLeft,vRight)
end
function turnLeft()
    vLeft=-2
    vRight=2
    setRobotVelo(vLeft,vRight)
end
function detect_GlobalPos(dista, angle)--variable
            
            --calculate sonarX and sonarY (into radians)
            robot_pos = sim.getObjectPosition(pioneer,-1)
            orientation = sim.getObjectOrientation(pioneer,-1)
            robot_heading = math.deg(orientation[3])
            
            --sonarX = math.cos(math.rad(sonarAngles[variable])) * (detect[variable] + robot_radius)
            --sonarY = math.sin(math.rad(sonarAngles[variable])) * (detect[variable] + robot_radius)--hyp
            sonarX = math.cos(math.rad(angle)) * (dista + robot_radius)
            sonarY = math.sin(math.rad(angle)) * (dista + robot_radius)--hyp
            
            --rotate by robot's heading            
            ga = orientation[3]
            cos_g = math.cos(ga)
            sin_g = math.sin(ga)
            
            rotatedX =(cos_g * sonarX) + (-(sin_g) * sonarY)
            rotatedY = (sin_g * sonarX) + (cos_g * sonarY)
            
            --translate by the robot's coordinates#
            globalX = robot_pos[1] + rotatedX
            globalY = robot_pos[2] + rotatedY
            globalPos = {globalX,globalY}
            globalPoint = {globalX, globalY, robot_pos[3]} --z could be anything as we ar not using that
            
            
            
            return globalPoint
end
function getDeci(x,place) -- set value to two decimal places
    deci = 10^place 
    return math.floor(x * deci)/ deci
end

function follow_path(array)
    check = false
    size = table.getn(array)
    res = move_to_point(array[1].x,array[1].y)
    if (res) then
        table.remove(array, 1)
    end
    
    if(size > 0)then
        check = false 
    else
        check = true
    end
    
    return check
end

function move_to_point(x,y)
    check=false
    
    distX = x - robot_pos[1]
    distY = y - robot_pos[2]    
    vector = {distX,distY}
    angleTo = math.atan2(distY,distX)
    angleTo = math.deg(angleTo)    
    --So we get angles from 0 to 360 (default: eg. -90 == 270)
    if(angleTo < 0)then
        angleTo = 180 + (180 + angleTo) 
    end
    --So we get angles from 0 to 360 (default: eg. -90 == 270)
    if(robot_heading <0)then
        robot_heading = 180 + (180 + robot_heading)
    end     
    --print("Heading: " .. robot_heading)
    --print("Angle to Point : " .. angleTo)
    distanceTo = math.sqrt(distX^2 + distY^2)    
    vL = v0 
    vR = v0
    print("[distance to point: " .. distanceTo .. " ]")
    if(distanceTo > 0.1)then        
        --g = math.abs(angleTo - robot_heading)
        error_direction=  robot_heading - angleTo
        --error_direction= math.abs(robot_heading - angleTo)        
        if(error_direction > 180)then
            error_direction = 180
        end
        
        if(angleTo <= (robot_heading + 180))then
            
            if(angleTo > robot_heading)then
                Prop = error_direction * 0.1
                
                vL = v0 + Prop
            elseif(angleTo + 180 < robot_heading) then                
                Prop = error_direction * 0.1
                            
                vR = v0 + Prop                
            else
                Prop = error_direction * 0.1
                
                vL = v0 + Prop
            end
        else --if(angleTo > (robot_heading + 180))then
            Prop = error_direction * 0.1
            vR = v0 + Prop
        end 
        moveNOW(vL,vR)
        check = false
        
    else
        check = true
    end    
    
    return check
    
end
function setHeading(ang)
     
    --So we get angles from 0 to 360 (default: eg. -90 == 270)
    if(ang < 0)then
        ang = 180 + (180 + ang) 
    end
    --So we get angles from 0 to 360 (default: eg. -90 == 270)
    if(robot_heading <0)then
        robot_heading = 180 + (180 + robot_heading) -- == 360 - robot_heading
    end     
      
    vL = v0 
    vR = v0
    
     
        --g = math.abs(angleTo - robot_heading)
        error_direction=  robot_heading - ang
        --error_direction= math.abs(robot_heading - angleTo)        
        if(error_direction > 180)then
            error_direction = 180
        end
        --print("------------------------heading error: " .. error_direction)        
        --print(robot_heading)
        if(ang <= (robot_heading + 180))then
            if(ang > robot_heading)then
                Prop = error_direction * 0.1
                vL = v0 + Prop
            elseif(ang + 180 < robot_heading) then                
                Prop = error_direction * 0.1               
                vR = v0 + Prop                
            else
                Prop = error_direction * 0.1
                vL = v0 + Prop
            end
        else --if(angleTo > (robot_heading + 180))then
            
            Prop = error_direction * 0.1
            vR = v0 + Prop
        end        
    
    moveNOW(vL,vR)
end

function PID(sensor)
            --error = target - actual
            error = setDist - sensor            
            --PROPORTIONAL
            p = (p_gain * error)            
            --INTEGRAL
            --store past n errors, average them, multiply avg by i_gain                       
            I_errors[I_count] = error
            I_count = I_count + 1
            
            if (I_count > I_max) then -- or use %I_max (modulo)
                I_sum = 0
                RMSE_sum = 0
                for i=1,I_max,1 do
                    I_sum = I_sum + I_errors[i]
                    RMSE_sum = RMSE_sum + (I_errors[i])^2
                    RMSE_avg = RMSE_sum/I_max
                    RMSE = math.sqrt(RMSE_avg) --RMSE_avg^0.5 --the smaller the value the better
                end
                I_Average = I_sum/I_max                 
                I_count = 1
            end
            i = i_gain * I_Average                  
            --DERIVATIVE
            D_error = previous_error - error
            previous_error = error --store error for next iteration            
            d = d_gain * D_error
             
            if (sensor == detect[8]) then
                vRight = v0 + p + i + d
            else
                vLeft = v0 + p + i + d
            end
end
function check_R(limit)
    limit = limit or setDist
    check = 0
    if((detect[7] < limit)
        or (detect[6] < limit)
        or (detect[5] < limit))then
        check = true
    else
        check = false
    end               

    return check
end
function check_L(limit)
    limit = limit or setDist
    check = 0
    if((detect[4] < limit)
        or (detect[3] < limit)
        or (detect[2] < limit))then
        check = true
    else
        check = false
    end               

    return check
end
function check_RL(limit)
    limit = limit or setDist
    check = 0
    if(check_R(limit) or check_L(limit))then
        check = true
    else
        check = false
    end               

    return check
end
function is_L_corner()
    check = 0
    if((detect[7] ~=10)
        and (detect[6] ~=10)
        and (detect[5] < 0.55)
        and (detect[4] < 0.55)
        and (detect[3] ~=10)
        and (detect[2] ~=10)
        and (detect[1] ~=10))then
        check = true
    elseif((detect[5] < 0.55)
        and (detect[4] < 0.55)
        and (detect[3] ~=10)
        and (detect[2] ~=10)
        and (detect[1] ~=10))then
        check = true
    
    else
        check = false
    end               

    return check
end

function update_Drawing_Grid()
    --Reset grid
    sim.addDrawingObjectItem(drawing_Grid_Occupied, nil)
    sim.addDrawingObjectItem(drawing_Grid_Free, nil)
    sim.addDrawingObjectItem(drawing_Grid_Unknown, nil)
    sim.addDrawingObjectItem(drawing_Grid_Occ, nil)
    sim.addDrawingObjectItem(drawing_Grid_Fr, nil)
    
    --Draw grid
    for i=1, grid_dimension do
        for j=1, grid_dimension do
            grid_point = {occupancy_grid[i][j].x, occupancy_grid[i][j].y, 15}--occupancy_grid[i][j].occupied}
            if(occupancy_grid[i][j].occupied == 1)then                
                sim.addDrawingObjectItem(drawing_Grid_Occupied, grid_point)
                
            elseif(occupancy_grid[i][j].occupied == 0)then                
                sim.addDrawingObjectItem(drawing_Grid_Free, grid_point)
                
            elseif(occupancy_grid[i][j].occupied == 0.75)then                
                sim.addDrawingObjectItem(drawing_Grid_Occ, grid_point)
                
            elseif(occupancy_grid[i][j].occupied == 0.25)then                
                sim.addDrawingObjectItem(drawing_Grid_Fr, grid_point)
                
            else                
                sim.addDrawingObjectItem(drawing_Grid_Unknown, grid_point)
            end
        end
    end
end

function update_Drawing_Grid_Point(i,j)
    
    grid_point = {occupancy_grid[i][j].x, occupancy_grid[i][j].y, 16}--occupancy_grid[i][j].occupied}
    if(occupancy_grid[i][j].occupied == 1)then                
        sim.addDrawingObjectItem(drawing_Grid_Occupied, {grid_point[1],grid_point[2],grid_point[3] + 0.1})
        
    elseif(occupancy_grid[i][j].occupied == 0)then                
        sim.addDrawingObjectItem(drawing_Grid_Free, {grid_point[1],grid_point[2],grid_point[3] + 0.2})
        
    elseif(occupancy_grid[i][j].occupied == 0.75)then                
        sim.addDrawingObjectItem(drawing_Grid_Occ, grid_point)
        
    elseif(occupancy_grid[i][j].occupied == 0.25)then                
        sim.addDrawingObjectItem(drawing_Grid_Fr, grid_point)
        
    else                
        sim.addDrawingObjectItem(drawing_Grid_Unknown, grid_point)
    end
   
end

function check_PointIn_Grid(point, weight, range)
    
    pointI = math.floor(point.x/steps + grid_shift/steps)+1
    pointJ = math.floor(point.y/steps + grid_shift/steps)+1
    
    
    
    iX = 0
    jY = 0
    grid_index = {}
    
    if(pointI < 1)then pointI = 1 
    elseif (pointI > grid_dimension)then  pointI = grid_dimension 
    end
    
    if(pointJ < 1)then pointJ = 1 
    elseif (pointJ > grid_dimension)then  pointJ = grid_dimension
    end
    
    --ind = 0
    for i=1, check_range do
        for j=1, check_range do
            
            iX = pointI - 1 + i
            jY = pointJ - 1 + j
            
            if(iX < 1)then                      iX = 1 
            elseif (iX > grid_dimension)then    iX = grid_dimension 
            end
            
            if(jY < 1)then                      jY = 1 
            elseif (jY > grid_dimension)then    jY = grid_dimension
            end
            
            --range = steps/2 --/2
            range_X = {x1 = occupancy_grid[iX][jY].x -range ,x2 = occupancy_grid[iX][jY].x +range}
            range_Y = {y1 = occupancy_grid[iX][jY].y -range ,y2 = occupancy_grid[iX][jY].y +range}
            
            if(point.x < range_X.x2 and
                point.x > range_X.x1)then
                if(point.y < range_Y.y2 and
                    point.y > range_Y.y1)then
                    --grid_index = {x = iX, y = jY}
                    
                    if(occupancy_grid[iX][jY].occupied == 1 or
                       occupancy_grid[iX][jY].occupied == 0 )then
                    else                        
                        occupancy_grid[iX][jY].occupied = weight
                        update_Drawing_Grid_Point(iX,jY)
                    end
                    
                end
            end
        end
    end
    
    --return grid_index

end

function get_PointIn_Grid(point)
    
    pointI = math.floor(point.x/steps + grid_shift/steps)+1
    pointJ = math.floor(point.y/steps + grid_shift/steps)+1   
    
    
    iX = 0
    jY = 0
    grid_index = {}
    
    if(pointI < 1)then pointI = 1 
    elseif (pointI > grid_dimension)then  pointI = grid_dimension 
    end
    
    if(pointJ < 1)then pointJ = 1 
    elseif (pointJ > grid_dimension)then  pointJ = grid_dimension
    end
    
    --ind = 0
    for i=1, check_range do
        for j=1, check_range do
            
            iX = pointI - 1 + i
            jY = pointJ - 1 + j
            
            if(iX < 1)then                      iX = 1 
            elseif (iX > grid_dimension)then    iX = grid_dimension 
            end
            
            if(jY < 1)then                      jY = 1 
            elseif (jY > grid_dimension)then    jY = grid_dimension
            end
            
            --range = steps/2 --/2
            range_X = {x1 = occupancy_grid[iX][jY].x -range ,x2 = occupancy_grid[iX][jY].x +range}
            range_Y = {y1 = occupancy_grid[iX][jY].y -range ,y2 = occupancy_grid[iX][jY].y +range}
            
            if(point.x < range_X.x2 and
                point.x > range_X.x1)then
                if(point.y < range_Y.y2 and
                    point.y > range_Y.y1)then
                    grid_index = {x = iX, y = jY}
                    return grid_index
                end
            end
        end
    end
    
    --return grid_index

end

function check_inCentre(centre, pos, range)
    
    check = false
    
    range_X = {x1 = centre.x -range ,x2 = centre.x +range}
    range_Y = {y1 = centre.y -range ,y2 = centre.y +range}
            
    if(pos.x < range_X.x2 and
        pos.x > range_X.x1)then
        if(pos.y < range_Y.y2 and
            pos.y > range_Y.y1)then
            check = true
        else
            check = false
        end
    else 
        check = false
    end
    
    
    return check
end



function search_Room_Centre()
        searchDist = 1
        --In this state do:
        if(search_stage == 0)then --search for left wall  
            print("[STAGE: 0")
            angl = math.floor((time_in_state * 5) % 180)
            print(angl)
            if(check_RL()) then
                turnRight()
                if(time_in_state > 5)then
                    if(robot_heading > (angl - 1) and robot_heading < (angl - 1)) then --89 91
                        search_stage = 1
                    --elseif(check_L())then
                      --  search_stage = 1
                    elseif(detect[1] ~=10 and detect[1]<searchDist)then 
                        search_stage = 1
                    end
                end
            else
                --angle
                setHeading(angl)--90--
                --vL=v0
                --vR=v0 
            end
        elseif(search_stage == 1)then --get to 2 corners
            print("[STAGE: 1 -")
            if(check_RL()) then
                turnRight()
            elseif(detect[1] ~= 10 and detect[1]<searchDist)then 
                    if(is_L_corner())then
                        rob = {robot_pos[1],robot_pos[2],17}
                        if(search_corner.c1.isCorner == false)then
                            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~corner ONE")
                            search_corner.c1.isCorner = true
                            search_corner.c1.x = robot_pos[1]
                            search_corner.c1.y = robot_pos[2]
                            sim.addDrawingObjectItem(drawing_SpecialPoints, rob)
                            corner_time = sim.getSimulationTime()
                        elseif(search_corner.c2.isCorner == false)then
                            
                            t = sim.getSimulationTime() - corner_time
                            if(t>10)then
                                print("~~~~~~~~~~~~~~~~~~~~~~FOUND~~~~~~~~~~~~~~~~~~~~~corner TWO")
                                search_corner.c2.isCorner = true
                                search_corner.c2.x = robot_pos[1]
                                search_corner.c2.y = robot_pos[2]
                                sim.addDrawingObjectItem(drawing_SpecialPoints, rob)
                                corner_time = sim.getSimulationTime()
                            else
                                print("~~~~~~~~~~~~~~~~~~~~~~SEARCHING~~~~~~~~~~~~~~~~~~~~~corner TWO")
                            end
                        elseif(search_corner.c3.isCorner == false)then
                            
                            t = sim.getSimulationTime() - corner_time
                            if(t>10)then
                                print("~~~~~~~~~~~~~~~~~~~~~~FOUND~~~~~~~~~~~~~~~~~~~~~corner THREE")
                                search_corner.c3.isCorner = true
                                search_corner.c3.x = robot_pos[1]
                                search_corner.c3.y = robot_pos[2]
                                sim.addDrawingObjectItem(drawing_SpecialPoints, rob)
                                corner_time = sim.getSimulationTime()
                            else
                                print("~~~~~~~~~~~~~~~~~~~~~~SEARCHING~~~~~~~~~~~~~~~~~~~~~corner THREE")
                            end
                        elseif(search_corner.c4.isCorner == false)then
                            
                            t = sim.getSimulationTime() - corner_time
                            if(t>10)then
                                print("~~~~~~~~~~~~~~~~~~~~~~FOUND~~~~~~~~~~~~~~~~~~~~~corner FOUR")
                                search_corner.c4.isCorner = true
                                search_corner.c4.x = robot_pos[1]
                                search_corner.c4.y = robot_pos[2]
                                sim.addDrawingObjectItem(drawing_SpecialPoints, rob)
                            else
                                print("~~~~~~~~~~~~~~~~~~~~~~SEARCHING~~~~~~~~~~~~~~~~~~~~~corner FOUR")
                            end
                        end
                    end
                    if(detect[2] ~= 10)then
                        PID(detect[1])
                        
                    else
                        vL = v0
                        vR = v0
                        --rob = {robot_pos[1],robot_pos[2],17}
                        --sim.addDrawingObjectItem(drawing_SpecialPoints, rob)
                    end
            else
                    setHeading(90)
                    vL=v0
                    vR=v0               
            end 
            if(search_corner.c4.isCorner)then
               
                x = (search_corner.c3.x + search_corner.c1.x)/2
                y = (search_corner.c3.y + search_corner.c1.y)/2
                
                Room_centre.x = x
                Room_centre.y = y
                
                sim.addDrawingObjectItem(drawing_SpecialPoints, {x,y,17})
                
                search_stage = 2
            end
            
        elseif(search_stage == 2)then --calculate centre and move to it
            print("[STAGE: 2 --")
            
            print("x: " .. Room_centre.x .. " |Y: " .. Room_centre.y )
            
            if(check_RL()) then
                turnRight()
            else
                move_to_point(Room_centre.x,Room_centre.y)
            end
            
            if(robot_pos[1] < Room_centre.x + 0.1 and
                        robot_pos[1] > Room_centre.x - 0.1)then
                        if(robot_pos[2] < Room_centre.y + 0.1 and
                            robot_pos[2] > Room_centre.y - 0.1)then
                           Room_centre.isVisited = true
                           search_stage = 3
                        end
            end
        elseif(search_stage == 3)then--exit the room
            print("[STAGE: 3 ---]")
            --moveNOW(0.1,0.1)
            setHeading(180)
            if(detect[4]<searchDist and
                detect[5]<searchDist)then
                task[1] = true
                search_stage = 0
            end
        end
end
function get_MapUnknown()
    notVisited = {}
    count = 0
    for i=1, grid_dimension do
        for j=1, grid_dimension do            
            if(occupancy_grid[i][j].occupied == 0.5)then
               count = count +1
            end
        end
    end
    
    return count
end
function check_MapComp(size, percentage)
    check = false
    unknowns = get_MapUnknown()
    percent = math.floor((unknowns/size) * 100)
    print("[Mapping: " .. percent .. "% left")
    
    val = size * percentage * 0.01    
    if (unknowns < val)then
        print("[Mapping: <20% --Good Enough]")
        check = true
    else
        check = false
    end
    
    return check    
end
function reset_aStar()
    openSet = {}
    closedSet = {}
    aPath = {}  
        sim.addDrawingObjectItem(drawing_Current, nil)
        sim.addDrawingObjectItem(drawing_Closed, nil)
        sim.addDrawingObjectItem(drawing_Open, nil)
      sim.addDrawingObjectItem(drawing_Path, nil)  
end
function aStar(start, goal, fun)
    fun = fun or false
    --start ->robot pos
    --goal ->centre of room
    --openSet = {}
    --closedSet = {}
    check = false
    
    if(start == goal and fun == false)then
        aPath = goal
        check = true
        print("you are at goal already?")
        return check
    end
    
    if(table.getn(openSet)== 0)then
        print("Starting A* search")
        current = start --node/point being evaluated
        current.h = get_hScore(current,start)
        current.g = get_gScore(current,goal)
        current.f = get_fScore(current.h,current.g)
        print("current: " .. current.f)
        
        openSet = {}
        closedSet = {}
        table.insert(openSet,current)
    
    
    elseif(table.getn(openSet)> 0) then
        
        --openSet = {}       
        --table.insert(openSet,current)
        table.remove(openSet)
        
        table.insert(closedSet, current)
        
        print("SEARCHING PATH ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        
        
        
        
        for i=1, 10 do --################# check 10 times
            --check current's neighbours    
            --print(i)
            if(current == 0 and table.getn(aPath)> 0)then
                check = false
                do return check end
            elseif(current.x == goal.x and
            current.y == goal.y )then
                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~FOUND PATH~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                --print(table.getn(current.parent))
                for i=1, table.getn(closedSet) do
                
                    print("--For loop:",i)
                    table.insert(aPath,1, current)
                    print("--Path inserted")
                    current = current.parent
                    --print("--Current = " , current)
                    if(current == start or current == 0)then
                        
                        sim.addDrawingObjectItem(drawing_Path, nil)  
                        for k,v in ipairs(aPath) do                            
                            if(aPath[k]~= 0) then
                                sim.addDrawingObjectItem(drawing_Path, {aPath[k].x, aPath[k].y,18.2})
                            end
                        end
                        print("--Path mapped")
                        check = true
                        do return check end
                    end
                    
                end
                --DraW
                sim.addDrawingObjectItem(drawing_Current, nil)                      
                sim.addDrawingObjectItem(drawing_Current, {current.x,current.y,18.3})
                
                
                
                
                sim.addDrawingObjectItem(drawing_Open, nil)  
                for i,v in ipairs(openSet) do              
                    sim.addDrawingObjectItem(drawing_Open, {openSet[i].x,openSet[i].y,18})
                end
                check = true   
                do return check end
            else 
                aPath = {}
            end 
            check_currentNeigh(current, start, goal) -- adds neigh to the open set
            minF = get_fMin_Set(openSet)--value == key to min fScore --gets the min fscore of open set
            nextMove(minF)--sets current node to the min fscore in openSet
        end
            --]]
            
            --print(closedSet)
        
        
        --DraW        
        sim.addDrawingObjectItem(drawing_Current, nil)                      
        sim.addDrawingObjectItem(drawing_Current, {current.x,current.y,18.2})
        
        
        sim.addDrawingObjectItem(drawing_Closed, nil)  
        for i,v in ipairs(closedSet) do              
            sim.addDrawingObjectItem(drawing_Closed, {closedSet[i].x,closedSet[i].y,18.1})
        end
        
        sim.addDrawingObjectItem(drawing_Open, nil)  
        for i,v in ipairs(openSet) do              
            sim.addDrawingObjectItem(drawing_Open, {openSet[i].x,openSet[i].y,18})
        end
        
        if(table.getn(aPath)>0)then
            check = true --path found
        else
            check = false
        end
        
        return check
    end
    
    
    sim.addDrawingObjectItem(drawing_Goal, nil)
    sim.addDrawingObjectItem(drawing_Goal, {goal.x,goal.y,18.2})
    
    
    
    
    
end



function get_gScore(current,start)
    gX = (start.x - current.x)^2
    gY = (start.y - current.y)^2    
    return math.sqrt(gX + gY) 
end

function get_hScore(current,goal)
    hX = (goal.x - current.x)^2
    hY = (goal.y - current.y)^2    
    return math.sqrt(hX + hY)
end

function get_fScore(hScore,gScore)
    return hScore + gScore
end

function check_currentNeigh(curr, start, goal)        
        for i,v in ipairs(curr.neigh) do
            --print("occ_grid index: " .. v[1],v[2])
            if(v[1]<1 or v[2]<1 or
               v[1]>grid_dimension or v[2]>grid_dimension )then
               --skip
            elseif(occupancy_grid[v[1]][v[2]].occupied > 0.75) then
                --go next
                --print("skip neigh")
            else                
                
                if(check_in_Set(closedSet, occupancy_grid[v[1]][v[2]]))then
                --skip
                else
                    --
                    X = occupancy_grid[v[1]][v[2]].x
                    Y = occupancy_grid[v[1]][v[2]].y 
                    occupancy_grid[v[1]][v[2]].h = get_hScore(occupancy_grid[v[1]][v[2]],goal)
                    occupancy_grid[v[1]][v[2]].g = get_gScore(occupancy_grid[v[1]][v[2]],start)
                    --
                    Gs = occupancy_grid[v[1]][v[2]].g
                    H = occupancy_grid[v[1]][v[2]].h
                    --
                    occupancy_grid[v[1]][v[2]].f = get_fScore(H,Gs)
                    F = occupancy_grid[v[1]][v[2]].f
                    --
                    newG = get_gScore(curr,start) + get_gScore(occupancy_grid[v[1]][v[2]],curr)
                    --
                    isInOpenSet = check_in_Set(openSet, occupancy_grid[v[1]][v[2]])
                    if(isInOpenSet)then
                        --Already in openSet
                    elseif(isInOpenSet == false or newG < Gs) then 
                        occupancy_grid[v[1]][v[2]].g = newG
                        Gs = occupancy_grid[v[1]][v[2]].g
                        H = occupancy_grid[v[1]][v[2]].h
                        occupancy_grid[v[1]][v[2]].f = get_fScore(H,Gs)
                        occupancy_grid[v[1]][v[2]].parent = current 
                        table.insert(openSet, occupancy_grid[v[1]][v[2]])
                    end
                    --sim.addDrawingObjectItem(drawing_Path, {X,Y,16})
                end
            end
            
        end
end        

function get_fMin_Set(set)
            fMin = {f = 999, key = 0}
            for i,v in ipairs(set) do                
                if (v.f < fMin.f or 
                   (v.f == current.f and v.h < current.h))then
                    fMin.f = v.f
                    fMin.key = i                    
                end
            end
            
            return fMin.key
end 

function check_in_Set(set , element)
            check = false
            for i,v in ipairs(set) do                
                if (v.x == element.x and
                    v.y == element.y)then
                    check = true
                    --print("element is already in set")
                end
            end
            return check
end 

function nextMove(key) 
        if(key == 0)then
            print("no path found? / no more nodes to check")
        else            
            current = openSet[key]    
            
            
            table.insert(closedSet,current)
            
            table.remove(openSet, key)
        end
end  

function lastFunction()
end

