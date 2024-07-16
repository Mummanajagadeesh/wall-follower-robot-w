"""wfr_py controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

def run_robot(robot):
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    
    #created motor instances 
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[ind].enable(timestep)
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    
        for ind in range(8):
            print(prox_sensors[ind].getValue())
            
        left_wall = prox_sensors[5].getValue() > 80
        front_wall = prox_sensors[7].getValue() > 80
        left_corner = prox_sensors[6].getValue() > 80
       
        
        
        left_speed = max_speed
        right_speed = max_speed
        
        if front_wall:
            print('Turn right in place')
            left_speed = max_speed
            right_speed = -max_speed
            
        else:
            if left_wall:
                print("Drive forward")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("Turn right")
                left_speed = max_speed / 8
                right_speed = max_speed 
            
            if left_corner:
                print("Came too close, drive right")
                left_speed = max_speed
                right_speed = max_speed / 8
                
            
            
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    
if _name_ == "_main_":
    my_robot = Robot()
    run_robot(my_robot)
