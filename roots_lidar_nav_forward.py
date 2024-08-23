import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32, Float64, Float32MultiArray, Float64MultiArray, Int32MultiArray
import math 
import numpy as np
from socket import *
import socket
import os        
import sys
import time
import sys, struct
import novatel_oem7_msgs
from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA
#from get_waypoints import *


get_lat_flag = 0
obst_det_flag = 0
ros_enable = 0


UDP_IP = "192.168.50.1"
UDP_PORT = 30000

hostName = gethostbyname('192.168.50.2')
portname = 51001

hostName2 = gethostbyname('127.0.0.1')
portname2 = 50000

global mySocket 

SIZE = 1024    # packet size

global steering_center
global steering_limits 
global center_threshold 
global code_start_flag

code_start_flag=1
steering_center= 350
steering_limits = 40
center_threshold = 25

global acceleration
global brake_1
global brake_2
global required_angle
global duty
global direction
global dist_next_wp
global send
global previous_steering

acceleration=0
brake_1=0
brake_2=0
required_angle=steering_center
duty=0
direction=0
send = [acceleration, brake_1, brake_2, required_angle, duty, direction]
previous_steering=0 

global last_bearing_diff
global steering_angle
global time1,time2
global lat, lng, heading, current_vel, vel_head

current_vel=0
steering_angle=0
#heading=0



global way_point_x
global way_point_y
global way_point_z
global heading 
global current_x
global current_y


def set_steering_angle_MABX(desired_angle):
    global required_angle
    required_angle = desired_angle + steering_center
    comm_MABX()

def fun_bearing_diff(Required_Bearing):
    global heading, Current_Bearing
    
    #get_heading()
    Current_Bearing = heading 
    while Current_Bearing is None:
       # get_heading()
        Current_Bearing = heading 
    
    Current_Bearing = float(Current_Bearing)
    #bearing_diff = Current_Bearing - Required_Bearing
    bearing_diff = -Current_Bearing + Required_Bearing
    #check this logic
    if bearing_diff < -180:
        bearing_diff = (bearing_diff+360)

    if bearing_diff > 180:
        bearing_diff = bearing_diff-360
    #    if abs(bearing_diff)>40:
        
    return bearing_diff
   
def callback_vel(data):
    global lat,lng,heading,current_vel,vel_head 
    global sol_status, pos_type, lat_delta, lng_delta, sat_used
    
    #rospy.loginfo(data)
    current_vel = 3.6 * data.hor_speed  
    vel_head = data.trk_gnd

rospy.Subscriber("/novatel/oem7/bestvel",BESTVEL, callback_vel)


def comm_MABX():
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global duty
    global direction
    global send
    global time1
    global time2
    global code_start_flag
    print("comm_MABX")
    print(time.time())
    time1= time.time()
    if (duty>80):
        duty=80
    duty = duty*100
##    print("duty X 100 = ", duty)
    send = [acceleration, brake_1, brake_2, required_angle,duty, direction]
    print(send)
    msg = []
    for i in range(len(send)):
       msg.append(struct.pack('!d',send[i])[: : -1])
    msg_string = b''.join(msg)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.sendto(msg_string,(UDP_IP,UDP_PORT))
    
    
    

def read_angle():
    global mySocket
    global steering_angle
    angle = steering_angle
##    mySocket  = socket(AF_INET,SOCK_DGRAM)
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    mySocket.bind((hostName,portname))
    mySocket.settimeout(0.2)
    repeat = True
    while repeat:
        try:
        ##    print("receiving")
            data,addr = mySocket.recvfrom(SIZE)
        ##    print("Received Data",data)
        ##    print("Size of data:", len(data))
            data = data[: : -1]
            if len(data) < 8:
                continue
            else:
                repeat=False
            angle = struct.unpack_from('!d',data)[0]
            print("Steering Angle: ",angle)
        except socket.timeout:
            print("closing socket")
            angle = steering_angle
            break  
    return (angle)
while (0):
   print("read_angle",read_angle())
    

def accelerate(speed1):
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global duty
    global direction
    global send
    global o_flag
    global time
    global time1
    global time2

    if (speed1>2.6):
        speed1=2.6
        
    acceleration = speed1
    
    #if(o_flag==1):
        #acceleration=0
    #time1=time.time()
    #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@time_difference",time1,time2)
    #if (time1-time2>0.4):
    comm_MABX() 
def apply_brake():
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global brake_flag
    global duty
    global direction
    global send
   
    brake_1=0
    brake_2=1
    acceleration = 0
    brake_flag=1
    comm_MABX()
    time.sleep(0.25)
    
    brake_1=0
    brake_2=0
    
    comm_MABX()
    
def remove_brake():
    global acceleration
    global brake_1
    global brake_2
    global required_angle
    global brake_flag
    global duty
    global direction
    global send
    
    brake_1=1
    brake_2=0
    
    comm_MABX()
    
    time.sleep(0.25)
    
    brake_1=0
    brake_2=0
    brake_flag=0
    comm_MABX()	
   
def set_velocity(desired_vel):
    global current_vel
    global acceleration
    

    desired_vel = float(desired_vel)
    current_vel = float(current_vel)

   # get_vel()
    current_vel = float(current_vel)
    
    print("  Desired Velocity       ",desired_vel)
    print("    Current velocity   ",current_vel)
    
    
    
    if desired_vel < current_vel:
        acceleration = acceleration - 0.1
    
        # if acceleration < 0:
        #     acceleration=0

        # el
        if acceleration<1:
            acceleration=1

    elif desired_vel > current_vel:
        acceleration = acceleration + 0.1
        
#         if acceleration < 1.5:
#             acceleration = 1.5
#         elif acceleration < 2:
#             acceleration = 2
#         elif acceleration < 2.5:
#             acceleration = 2.5
#         else:
#             acceleration = acceleration + 0.5
            
#         if acceleration > (1.75 + desired_vel-current_vel):
#             acceleration = (1.75 + desired_vel-current_vel)
    if current_vel < 3 and acceleration > 1.5:
        acceleration = 1.5
        
    accelerate(acceleration)
    return current_vel



current_x=0
current_y=0   

def callback_ndt_pose(data):
   global current_x
   global current_y
   current_x = data.pose.position.x
   current_y = data.pose.position.y 
   #print("current_x", current_x)
   #print("current_y",current_y)
   #distance = math.sqrt((current_x - way_point_x) ** 2 + (current_y - way_point_y) ** 2)
   
   #print("distance between current pos and waypoint is ", distance)
   
        
   
      
      
   
   


heading=0
def callback_eular_angle(data):
   global heading
   heading = data.data[2] * (180 / np.pi) 
  # print("current heading is", heading)
   
   
   


def listener():
   rospy.init_node("topic_subscriber")
   
   
   rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)
   rospy.Subscriber("/eular_angle", Float32MultiArray, callback_eular_angle)
   
  # rospy.spin()
  
  
#way_point_x = 25.690
#way_point_y = 0.515

waypoints=[[]]

#waypoints= waypoints_forward

global wp
wp=0
if __name__ == "__main__" :
    current_steering_topic = rospy.Publisher('current_steering_angle', Float32, queue_size=1)
    required_steering_topic = rospy.Publisher('required_steering_angle', Float32, queue_size=1)
    required_bearing_ppc_topic = rospy.Publisher('required_beering_ppc', Float32, queue_size=1)
    current_bearing_diff_topic = rospy.Publisher('current_bearing_diff', Float32, queue_size=1)
    steer_output_topic = rospy.Publisher('steer_output', Float32, queue_size=1)
    velocity_topic = rospy.Publisher('velocity_topic', Float32, queue_size=1)
    listener()
    while(1):
       print("current heading is", heading)
       print("current_x", current_x)
       print("current_y",current_y)
       #distance = math.sqrt((current_x - way_point_x) ** 2 + (current_y - way_point_y) ** 2)
   
       
       
   #waypoints= [9, -5]
   
   
       distance = math.sqrt((current_x - waypoints[wp][0]) ** 2 + (current_y - waypoints[wp][1]) ** 2)
   
       print("distance between current pos and waypoint is ", distance)
       if (wp == len(waypoints)-1):
          set_velocity(0)
          apply_brake()
          time.sleep(0.25)
          apply_brake()
          time.sleep(0.25)
          apply_brake()
          time.sleep(0.25)
          apply_brake()
          time.sleep(0.25)
          remove_brake()
          time.sleep(0.25)
          remove_brake()
          time.sleep(0.25)
          remove_brake()
          time.sleep(0.25)
          remove_brake()
          break
       else:
          if (wp < len(waypoints)):
##                
             Ld_steer =8
      
             off_y = - current_y + waypoints[wp][1]
             off_x = - current_x + waypoints[wp][0]

      #bearing_ppc =90+math.atan2(-off_y, off_x) * 57.2957795
             bearing_ppc = math.atan2(off_y, off_x) * 57.2957795
             steering_angle = read_angle()
             
             current_steering_topic.publish(steering_angle)  #Current steering angle
             
             if bearing_ppc < 0:
                bearing_ppc += 360.00
        
             print("required bearing ppc = ",bearing_ppc)
             
             required_bearing_ppc_topic.publish(bearing_ppc)

             current_bearing_diff_ppc = fun_bearing_diff(bearing_ppc)
             steer_output = 750 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * current_bearing_diff_ppc / 180) / Ld_steer  )
   #set_velocity(5)
             print("steer_output", steer_output)
             steer_output_topic.publish(steer_output)
             print("current_bearing_difference",current_bearing_diff_ppc)
             current_bearing_diff_topic.publish(current_bearing_diff_ppc)
             print("****************************************************required steer angle = ",steer_output + steering_center)
             required_steering_topic.publish(steer_output + steering_center)
             print("****************************************************Current steering angle = ",steering_angle)
             print("****************************************************steering angle difference = ", (steer_output + steering_center) - steering_angle)
             current_vel=set_velocity(5) 
             velocity_topic.publish(current_vel)
                  
             set_steering_angle_MABX(steer_output)
             Ld=3
             print("wp : ",wp)               
             if (distance < Ld ) and (wp<len(waypoints)):
                print("################################################# WP Changed")
                wp=wp+1
       time.sleep(0.1)
