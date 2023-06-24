#!/usr/bin/python3
import serial 
import time
import numpy as np
import math

Kp = 0.5  # Proportional gain
Ki = 0.2  # Integral gain
Kd = 0.1  # Derivative gain
a_MAX = 3
wheelRadius = (37.5/2)/1000
pi = math.pi
DISTANCE_TH = 2
THETA_FACTOR = 3
DE_FACTOR = 1


MIN_SPEED = -400
MAX_SPEED = 400
REFERENCE_SPEED = 200
STOP_THRESHOLD = 0.1


## Omri - I've created a new function that looks more like what we have in the matlab. only to try some things.
def P2P_CTRL_NEW(DesiredPos, CurrPos, PHI, a_Max):
    global currPoint
    global dist_so_far
    global dist_total
    global last_x, last_y

    if 'dist_so_far' not in globals():
        print("Sets dist so far to 0")
        dist_so_far = 0
    
    if 'currPoint' not in globals():
        print("Sets curr Point to 0")
        currPoint = 0

    if 'dist_total' not in globals():
        dist_total = np.linalg.norm(DesiredPos[currPoint, :] - CurrPos)
        print(f"Sets dist_total Point to {dist_total}")
    
    if 'last_x' not in globals():
        last_x = 0
    if 'last_y' not in globals():
        last_y = 0

    Finished = 0
    
    dist_so_far = dist_so_far + np.linalg.norm(np.array([last_x,last_y]) - CurrPos) 

    print (f"dist_so_far-{dist_so_far}")
    # calculate car vectors and Path vectors - What are those? 
    Vr = np.array([np.cos(PHI), np.sin(PHI)])  # car direction vector - radios?
    Vt = DesiredPos[currPoint, :] - CurrPos  # car direction vector - angle? 
    print(f"Vr - {Vr} , Vt - {Vt}")

    # Cross and dot vectors 
    crossVector = Vt[1]*Vr[0] - Vt[0]*Vr[1] 
    print(f"np.linalg.norm(Vt) - {np.linalg.norm(Vt)} , np.linalg.norm(Vr) - {np.linalg.norm(Vr)}")
    dotVector = np.dot(Vt, Vr) / (np.linalg.norm(Vt) * np.linalg.norm(Vr))


    print(f"Cross vector - {crossVector} , Dot Vector - {dotVector}")
    dir = 0
    # Calculate the desired angle change for the target point 
    if np.abs(crossVector) < 0.001:  # Math singularity
        if dotVector == -1:  # vectors align in reverse... singularity
            theta_t = np.pi/2
            print("Reached theta_t = np.pi/2")
        else:  # Vectors align in the same direction
            theta_t = 0
            print("Reached theta_t = 0")
    else:
        print("Reached the else part in the angle shit")
        dir = np.sign(crossVector)
        theta_t = np.arccos(dotVector) * dir  # angle from desired path
    de = crossVector / np.linalg.norm(Vt)  # distance to desired path

    print(f"dir - {dir} , theta_t - {theta_t}, de - {de}")

    # Calculate distance to last point: 
    dist = np.linalg.norm(Vt)  # distance to current point
    pathDist = 0  # remaining points distance
    numPoints = DesiredPos.shape[0] # Gives us the number of point in the array. 

    print(f"dist - {dist} , pathDist - {pathDist}, numPoints - {numPoints}")

    if currPoint < numPoints:
        print(f"Reached point number {currPoint}")
        for i in range(currPoint, numPoints-1):
            pathDist += np.sqrt(np.sum((DesiredPos[i+1, :] - DesiredPos[i, :])**2))
            print(f"pathDist for point {i+1} - {pathDist}")
        
        # pass to next point condition
        if dist_total - DISTANCE_TH< dist_so_far < dist_total + DISTANCE_TH: ## Copied from MATLAB  - dont understand why? 
            if currPoint + 1 == numPoints:
                Finished = 1
                V_Forward = 0
                theta_t = 0
                En = 0
                print("Reached finish condition ")
            else:
                currPoint += 1
                dist_so_far = 0
                dist_total = np.linalg.norm(DesiredPos[currPoint, :] - CurrPos)

            print(f"Reached the loop where we increase the point number | set dist_total to {dist_total} ")
            print(f"currPoint- {currPoint}, numPoints- {numPoints} ")

        # update remaining distance
        dist += pathDist
        print(f"Remaining dist = {dist}")

        # set desired velocity: 
        if (abs(dist)<DISTANCE_TH):
            V_Forward = 0
            theta_t = 0
            En = 0
            Finished = 1
        
        else:
            V_Forward = np.sqrt(2*a_MAX*dist)
            En = 1
        
        print(f"V_forward - {V_Forward} , En - {En}")
        last_x, last_y = CurrPos
        return V_Forward, theta_t, de, dist, En, Finished
        

def pid_controller(error, integral, derivative):
    return Kp * error + Ki * integral + Kd * derivative

def control_loop(target_velocity, actual_velocity, dt):
    # Initialize variables
    integral = 0.0
    prev_error = 0.0
    control_signal = []
    
    # Convert single values to arrays
    if not isinstance(target_velocity, np.ndarray):
        target_velocity = np.array([target_velocity])
    if not isinstance(actual_velocity, np.ndarray):
        actual_velocity = np.array([actual_velocity])
    
    # Control loop
    for i in range(len(target_velocity)):
        error = target_velocity[i] - actual_velocity[i]
        
        # Calculate the integral term
        integral += error * dt
        
        # Calculate the derivative term
        derivative = (error - prev_error) / dt
        
        # Compute the control signal using the PID controller
        control = pid_controller(error, integral, derivative)
        
        # Store the control signal
        control_signal.append(control)
        
        # Update previous error
        prev_error = error
    
    return control_signal

def control_loop2(target_velocity, actual_velocity, dt):
    # Initialize variables
    integral = 0.0
    prev_error = 0.0
    control_signal = []

    # Convert single values to arrays
    if not isinstance(target_velocity, np.ndarray):
        target_velocity = np.array([target_velocity])
    if not isinstance(actual_velocity, np.ndarray):
        actual_velocity = np.array([actual_velocity])

    # Control loop
    for i in range(len(target_velocity)):
        error = target_velocity[i] - actual_velocity[i]

        # Calculate the integral term
        integral += error * dt

        # Calculate the derivative term
        derivative = (error - prev_error) / dt

        # Compute the control signal using the PID controller
        control = pid_controller(error, integral, derivative)

        # Store the control signal
        control_signal.append(control)

        # Update previous error
        prev_error = error

    return control_signal

def parse_arduino_line():
    line = ser.readline().decode('utf-8')
    params = line.split(',')
    params = [float(param.strip()) for param in params]
    return params 

               
def print_arduino_line():  
    line = ser.readline().decode('utf-8')
    params = line.split(',')
    params = [float(param.strip()) for param in params]

    headers = ["Left Motor", "Right Motor", "Battery", "dt_time", "pos x", "pos y", "ody ang", "gyro ang"]
    col_widths = [max(len(header), len(str(param))) for header, param in zip(headers, params)]

    print("|".join([f"{header:<{col_widths[i]}}" for i, header in enumerate(headers)]))
    print("|".join([f"{param:<{col_widths[i]}.2f}" for i, param in enumerate(params)]))

def turn_degrees(curr_degrees, desired_degrees):
    print(f"turn to degree- {desired_degrees}")
    degree_total = curr_degrees + desired_degrees
    while (curr_degrees < degree_total):
        msg_to_motor = str(-350) + ',' + str(+180) + '\r\n'
        ser.write(msg_to_motor.encode('ascii'))
        left_motor, right_motor, battery, dt_time, pos_x, pos_y, ody_ang , gyro_ang = parse_arduino_line() 
        curr_degrees = ody_ang
        
    msg_to_motor = str(0) + ',' + str(0) + '\r\n'
    ser.write(msg_to_motor.encode('ascii'))
        
    




if __name__ == "__main__":

    ser = serial.Serial('/dev/ttyACM0')
    time.sleep(1)
    global ref_x
    global ref_y
    global ref_ang
    ref_x = 0
    ref_y = 0
    ref_ang = 0
    
    # path_points = [
    #             [50, 0]]
    # path_points = np.array(path_points)

    path_points = np.array([[i, 0] for i in range(50, 100, 30)])
    # # Define the coordinates for the square
    top_line = np.array([[ 50,   0],
                        [100,   0],
                        [150,   0],
                        [200,   0],
                        [250,   0]])
    bottom_line = np.array([[ 250,   50],
                        [250,   100],
                        [250,   150],
                        [250,   200],
                        [250,   250]])
    right_line = np.array([[ 200,   250],
                        [150,   250],
                        [100,   250],
                        [50,   250],
                        [0,   250]])
    left_line = np.array([[ 0,   200],
                        [0,   150],
                        [0,   100],
                        [0,   50],
                        [0,   0]])

    # # Concatenate the lines to form the square
    # path_points = np.concatenate((top_line, right_line, bottom_line, left_line))

    # path_points = np.array([[i, 0] for i in range(50, 300, 50)])

    line = top_line
    
    K = 1/(wheelRadius*2*pi)

    msg_to_motor = str(0) + ',' + str(0) + '\r\n'       # Why? 

    t = np.linspace(0, 10, num=100)
    dt = t[1] - t[0]
    ser.write(msg_to_motor.encode('ascii'))
    print_arduino_line()
    ser.write(msg_to_motor.encode('ascii'))
    turn_point = [70,65,60]
    for i in range(4):
        finished = 0
        print(f"Reached {line}")
        msg_to_motor = str(0) + ',' + str(0) + '\r\n'
        ser.write(msg_to_motor.encode('ascii'))
        while finished == 0:
            left_motor, right_motor, battery, dt_time, pos_x, pos_y, ody_ang , gyro_ang = parse_arduino_line() 
            # pos_x = pos_x - ref_x
            # pos_y = pos_y - ref_y
            # ody_ang = ody_ang - ref_ang
            print(f"Update pos_x and pos_y: ({pos_x},{pos_y}) | angle - {ody_ang}")
            V_Foword, theta_t, de, dist, En, finished = P2P_CTRL_NEW(line,[pos_x,pos_y], ody_ang, a_MAX)
            print(f"({pos_x},{pos_y})")
            print(f"({theta_t*0.1 + de*0.25})")
            target_velocity_left = K*(V_Foword - (theta_t*THETA_FACTOR + de*DE_FACTOR)) 
            target_velocity_right = K*(V_Foword + (theta_t*THETA_FACTOR + de*DE_FACTOR)) 

            # Calculate the scaling factor based on the ratio with the reference speed
            ratio = target_velocity_left / target_velocity_right
            scaling_factor = abs(REFERENCE_SPEED) / max(abs(target_velocity_left), abs(target_velocity_right))

            # Normalize the target velocities around the reference speed
            target_velocity_left *= scaling_factor
            target_velocity_right *= scaling_factor

            # Check if the velocities are below the stop threshold
            if abs(target_velocity_left) < STOP_THRESHOLD and abs(target_velocity_right) < STOP_THRESHOLD:
                target_velocity_left = 0
                target_velocity_right = 0
            else:
                # Apply speed limits
                if target_velocity_left > MAX_SPEED:
                    target_velocity_left = MAX_SPEED
                    target_velocity_right = target_velocity_left / ratio
                elif target_velocity_left < -MAX_SPEED:
                    target_velocity_left = -MAX_SPEED
                    target_velocity_right = target_velocity_left / ratio
                elif target_velocity_left > 0 and target_velocity_left < MIN_SPEED:
                    target_velocity_left = MIN_SPEED
                    target_velocity_right = target_velocity_left / ratio
                elif target_velocity_left < 0 and target_velocity_left > -MIN_SPEED:
                    target_velocity_left = -MIN_SPEED
                    target_velocity_right = target_velocity_left / ratio

                if target_velocity_right > MAX_SPEED:
                    target_velocity_right = MAX_SPEED
                    target_velocity_left = target_velocity_right * ratio
                elif target_velocity_right < -MAX_SPEED:
                    target_velocity_right = -MAX_SPEED
                    target_velocity_left = target_velocity_right * ratio
                elif target_velocity_right > 0 and target_velocity_right < MIN_SPEED:
                    target_velocity_right = MIN_SPEED
                    target_velocity_left = target_velocity_right * ratio
                elif target_velocity_right < 0 and target_velocity_right > -MIN_SPEED:
                    target_velocity_right = -MIN_SPEED
                    target_velocity_left = target_velocity_right * ratio
                
            if finished ==1:
                msg_to_motor = "0" + ',' + "0" + '\r\n'
                ser.write(msg_to_motor.encode('ascii'))
                print("While loop ended!\n\n")
                currPoint = 0 
                dist_so_far = 0 
                dist_total = 0
                last_x, last_y
            else:
                print(f"target_velocity_left - {target_velocity_left} | target_velocity_right - {target_velocity_right}")
                msg_to_motor = str(target_velocity_left) + ',' + str(target_velocity_right) + '\r\n'
                ser.write(msg_to_motor.encode('ascii'))
                print("\n\n") 


            # control_velocity_left = control_loop2(target_velocity_left, left_motor,dt)
            # control_velocity_right = control_loop2(target_velocity_right, right_motor,dt)
        left_motor, right_motor, battery, dt_time, pos_x, pos_y, ody_ang , gyro_ang = parse_arduino_line() 
        turn_degrees(ody_ang,turn_point[i])
        print("Turn!!")
        msg_to_motor = "0" + ',' + "0" + '\r\n'
        ser.write(msg_to_motor.encode('ascii'))
        left_motor, right_motor, battery, dt_time, pos_x, pos_y, ody_ang , gyro_ang = parse_arduino_line() 
        ref_x = pos_x
        ref_y = pos_y
        last_x, last_y = pos_x, pos_y
        ref_ang = ody_ang
        print(f"ref_x - {ref_x} | ref_y - {ref_y} | ref_ang - {ref_ang}")
        msg_to_motor = "0" + ',' + "0" + '\r\n'
        ser.write(msg_to_motor.encode('ascii'))
        
        # # Create new line 
        # if i == 0:
        #     line = np.array([[ref_x, ref_y + j] for j in range(int(ref_y), int(ref_y)+300, 50)])
        # if i == 1: 
        #     line = np.array([[ref_x + j, ref_y] for j in range(int(ref_x), int(ref_x)-300, -50)])
        # if i == 2: 
        #     line = np.array([[ref_x , ref_y +j] for j in range(int(ref_y)-300, int(ref_y)-300, -50)])
        
        # Create new lines
        if i == 0:
            line = np.array([[ref_x, y] for y in range(int(ref_y), int(ref_y) + 150, 30)])


        if i == 1:
            line = np.array([[x, ref_y] for x in range(int(ref_x), int(ref_x) - 150, -30)])


        if i == 2:
            line = np.array([[ref_x, y] for y in range(int(ref_y), int(ref_y) - 150, -30)])

        
        j = 0
        print(f"i- {i} - New Line created: {line}")
        

print("Got out of the for")



        


