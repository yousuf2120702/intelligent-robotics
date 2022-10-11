from controller import Motor, Supervisor
import numpy as np
import math, time

TIME_STEP = 1000
MAX_SPEED = 6.28

super = Supervisor()
goal = np.array([0.45,0.0])
source = np.array([-1.13, 0.003,0.000])
robot = super.getFromDef('epuck') 
epuck_orientation = robot.getOrientation()
heading_angle = np.arctan2(epuck_orientation[0], epuck_orientation[3])
heading_vec = np.array([np.cos(heading_angle),np.sin(heading_angle)])

leftMotor = super.getDevice('left wheel motor')
rightMotor = super.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


print(heading_angle)
print(heading_vec)

def angleWrap(ang):
    while ang>=2*np.pi:
        ang -= 2*np.pi
    while ang<0:
        ang += 2*np.pi
    return ang

def rotation_controller(desired_angle):
    # print('here',desired_angle)
    global leftMotor, rightMotor, heading_angle, robot, super
    reached = 0
    accuracy = 0.5
    while super.step(TIME_STEP) != -1:

        epuck_orientation = robot.getOrientation()
        heading_angle = np.arctan2(epuck_orientation[0], epuck_orientation[3])
        heading_vec = np.array([np.cos(heading_angle),np.sin(heading_angle)])
        
        desired_vec = np.array([np.cos(desired_angle),np.sin(desired_angle)])
        rotation_dir = np.cross(heading_vec,desired_vec)
        rotation_dir = rotation_dir/abs(rotation_dir)
        
        theta = np.arccos(np.dot(desired_vec,heading_vec))
        # print('printing theta',theta)
        if theta*180/np.pi > accuracy:
            # print('here') 
            leftSpeed = rotation_dir * abs(heading_angle - desired_angle)
            rightSpeed = -rotation_dir * abs(heading_angle - desired_angle)#* MAX_SPEED/abs(heading_angle - desired_angle)
        else:
            leftSpeed = 0.0
            rightSpeed = 0.0
        print([leftSpeed,rightSpeed,rotation_dir,desired_angle,heading_angle,heading_angle - desired_angle])
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        if abs(heading_angle - desired_angle)*180/np.pi <= accuracy:
            # print('printing rotation error',[heading_angle, desired_angle])
            reached = 1
            break
    return reached
            
def translation_controller(goal):
    global leftMotor, rightMotor, robot, super, ps
    accuracy = 72.54

    while super.step(TIME_STEP) != -1:
        back_obstacle = (ps[3].getValue() > accuracy or ps[4].getValue() >accuracy)
        front_obstacle =  (ps[7].getValue() > accuracy and ps[0].getValue() >accuracy)
        right_obstacle = (ps[1].getValue() > accuracy or ps[2].getValue() > accuracy)
        left_obstacle = (ps[5].getValue() > accuracy)
        current_location = robot.getField('translation')
        current_location = current_location.getSFVec3f()
        desired_dir = goal - np.array(current_location)[:2]
        # print(desired_dir)
        # Desired direction needs some correction, need to debug
        desired_dir_norm = np.linalg.norm(desired_dir)
        epuck_orientation = robot.getOrientation()
        heading_angle = np.arctan2(epuck_orientation[0],epuck_orientation[3])
        heading_vec = np.array([np.cos(heading_angle),np.sin(heading_angle)])
        
        if desired_dir_norm!=0:
            desired_dir = desired_dir/desired_dir_norm
        desired_ang = np.arctan2(desired_dir[0], desired_dir[1])
        
        # rotation_controller(np.pi/2)
        if front_obstacle:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            # ## After correcting desired direction, here need to make change how much to rotate
            desired_angle = 0.0
            # print('front_obs')
            right_obs = (ps[2].getValue() > accuracy)
            while not right_obs:
                epuck_orientation = robot.getOrientation()
                heading_angle = np.arctan2(epuck_orientation[0],epuck_orientation[3])
                reached = 0
                if not reached:
                    if ps[0].getValue() > 70:
                        reached = rotation_controller(heading_angle-np.pi/2)
                    else:
                        reached = rotation_controller(heading_angle+np.pi/2)
                right_obs = (ps[2].getValue() > accuracy)
            back_obstacle = (ps[3].getValue() > accuracy and ps[4].getValue() >accuracy)
            if back_obstacle:
                desired_angle = np.pi/2
                # print('back_obs')
                reached = 0
                if not reached:
                    reached = rotation_controller(desired_angle)

        elif right_obstacle:
            # print('right_obs')
            leftSpeed = 0.3 * MAX_SPEED
            rightSpeed = 0.3 * MAX_SPEED
        
        else:
            # ## theta = np.arccos(np.dot(desired_dir,heading_vec))
            # print('no obstacle')

            reached = 0
            if not reached:
                # ## reached = rotation_controller(theta)
                reached = rotation_controller(desired_ang)
                # print('breaking', reached)
            if reached:
                # print('heading towards the goal')
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                
        if desired_dir_norm <=0.1:
            # print('bad breaking')
            break

        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        

desired_angle = heading_angle
count = 0
#initialize devices
ps = []
for i in range(8):
    ps.append(super.getDevice('ps'+str(i)))
    ps[i].enable(TIME_STEP)
# while super.step(TIME_STEP) != -1:
epuck_orientation = robot.getOrientation()
heading_angle = np.arctan2(epuck_orientation[0], epuck_orientation[3])
current_location = robot.getField('translation')
current_location = current_location.getSFVec3f()

translation_controller(goal)
