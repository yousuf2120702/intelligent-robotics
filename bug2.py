from controller import Motor, Supervisor
import numpy as np
import math, time

TIME_STEP = 1000
MAX_SPEED = 6.28

super = Supervisor()
goal = np.array([0.45,0.0])
source = np.array([-1.13, 0.003])
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

def saturate(value,limit):
    if abs(value) > limit:
        value = limit*value/abs(value)
    return value
    
def rotation_controller(desired_angle):
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
        if theta*180/np.pi > accuracy:
            leftSpeed = rotation_dir * saturate(abs(heading_angle - desired_angle),6)
            rightSpeed = -rotation_dir * saturate(abs(heading_angle - desired_angle),6)
        else:
            leftSpeed = 0.0
            rightSpeed = 0.0
        print([theta*180/np.pi,heading_angle,desired_angle])
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        if theta*180/np.pi <= accuracy:
            reached = 1
            print('oriented')
            break
    return reached
            
def translation_controller(goal):
    global leftMotor, rightMotor, robot, super, ps
    accuracy = 75
    previous_state = None
    while super.step(TIME_STEP) != -1:
        back_obstacle = (ps[3].getValue() > accuracy or ps[4].getValue() >accuracy)
        front_obstacle =  (ps[7].getValue() > accuracy and ps[0].getValue() >accuracy)
        right_obstacle = (ps[1].getValue() > accuracy or ps[2].getValue() > accuracy)
        left_obstacle = (ps[5].getValue() > accuracy)
        current_location = robot.getField('translation')
        current_location = current_location.getSFVec3f()
        desired_dir = goal - np.array(current_location)[:2]

        desired_dir_norm = np.linalg.norm(desired_dir)
        epuck_orientation = robot.getOrientation()
        heading_angle = np.arctan2(epuck_orientation[0],epuck_orientation[3])
        heading_vec = np.array([np.cos(heading_angle),np.sin(heading_angle)])
        
        if desired_dir_norm!=0:
            desired_dir = desired_dir/desired_dir_norm
        desired_ang = np.arctan2(desired_dir[0], desired_dir[1])
        
        if front_obstacle:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            desired_angle = 0.0
            print('front_obs')
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
            previous_state = 'f'

        elif right_obstacle:
            source_dir = source - np.array(current_location)[:2]
            goal_dir = goal - np.array(current_location)[:2]
            source_dir_norm = np.linalg.norm(source_dir)
            goal_dir_norm = np.linalg.norm(goal_dir)
            if source_dir_norm!=0:
                source_dir = source_dir/source_dir_norm
            if goal_dir_norm!=0:
                goal_dir = goal_dir/goal_dir_norm
            print('m_line_error',abs(np.arccos(np.dot(goal_dir,source_dir)) - np.pi)*180/np.pi)
            if abs(np.arccos(np.dot(goal_dir,source_dir)) - np.pi)*180/np.pi>2:#perpendicular_distance_from_m_line>0.1 or abs(heading_angle - desired_ang)*180/np.pi>10:
                leftSpeed = 0.3 * MAX_SPEED
                rightSpeed = 0.3 * MAX_SPEED
                previous_state = 'r'
            else:
                leftSpeed = 0.
                rightSpeed = 0.
                reached = 0
                if not reached:
                    reached = rotation_controller(desired_ang)
                if reached:
                    # print('heading towards the goal')
                    leftSpeed = 0.5 * MAX_SPEED
                    rightSpeed = 0.5 * MAX_SPEED
                previous_state = None
            
            
        elif previous_state == 'r' and (not front_obstacle) and (not right_obstacle):
            previous_state = 'o'
            print('entering here',)
            reached = 0
            if not reached:
                reached = rotation_controller(heading_angle + np.pi/2)
        
        elif previous_state!='o':
            print('no obstacle')
            # Make changes here for bug 2
            m_line = goal - source
            source_current_vec = np.array(current_location)[:2] - source
            # print('source_current_vec',source_current_vec)
            perpendicular_distance_from_m_line = np.linalg.norm(np.cross(source_current_vec,m_line))/np.linalg.norm(m_line)
            
            if perpendicular_distance_from_m_line>0.01:
                mline_angle = np.arctan2(m_line[0],m_line[1])
                # print('mline_angle',mline_angle)
                source_cu = np.arccos(np.dot(m_line/np.linalg.norm(m_line),source_current_vec/np.linalg.norm(source_current_vec)))
                # print('source_cu',source_cu)
                heading_so_cu = np.arccos(np.dot(heading_vec,source_current_vec/np.linalg.norm(source_current_vec)))
                heading_so_cu_rot = np.cross(heading_vec,source_current_vec/np.linalg.norm(source_current_vec))
                if heading_so_cu_rot!=0:
                    heading_so_cu_rot = heading_so_cu_rot/abs(heading_so_cu_rot)
                # print('heading_so_cu',heading_so_cu)
                desired_ang = np.pi/2 + mline_angle + source_cu - heading_so_cu#np.pi/2 + mline_angle + source_cu + heading_so_cu_rot*heading_so_cu
                print('desired_ang',desired_ang)
                reached = 0
                if not reached:
                    print('Orienting towards M-Line')
                    reached = rotation_controller(desired_ang)
                if reached:
                    # print('heading towards the goal')
                    leftSpeed = 0.5 * MAX_SPEED
                    rightSpeed = 0.5 * MAX_SPEED
            else:
                # else:
                reached = 0
                if not reached:
                    reached = rotation_controller(desired_ang)
                if reached:
                    # print('heading towards the goal')
                    leftSpeed = 0.5 * MAX_SPEED
                    rightSpeed = 0.5 * MAX_SPEED
                
        if desired_dir_norm <=0.1:
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
epuck_orientation = robot.getOrientation()
heading_angle = np.arctan2(epuck_orientation[0], epuck_orientation[3])
current_location = robot.getField('translation')
current_location = current_location.getSFVec3f()

translation_controller(goal)
