#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist
import time
import numpy as np



velocity_message = Twist()

def scan_callback(scan_data):
   
    #angle_inc= scan_data.angle_increment
    #index1= np.math.ceil(7/180*3.1415/angle_inc)
    #index2= np.math.ceil((360-7)/180*3.1415/angle_inc)

    global critical_distance, critical_left, critical_right,critical_left_corner,critical_right_corner,back_clearance

    critical_distance, _, _= average_between_indices(scan_data.ranges, 5, 355)
    #critical_left= average_sideways(scan_data.ranges, 85, 95)
    #critical_right= average_sideways(scan_data.ranges, 265, 275)
    critical_left_corner= average_sideways(scan_data.ranges, 42, 48)
    critical_right_corner= average_sideways(scan_data.ranges, 312, 318)
    back_clearance= average_sideways(scan_data.ranges, 177-10, 183+10)


    print ("\nCritical Distance: ", critical_distance)
    print ("\nCritical Left Corner: ", critical_left_corner)
    print ("\nCritical Right Corner: ", critical_right_corner)
    time.sleep(0.1)




def move_straightline(velocity_publisher, speed):
   
    delay_obj= rospy.Rate(20)
    velocity_message.linear.x =speed
    velocity_publisher.publish(velocity_message)
    delay_obj.sleep()




def rotate(velocity_publisher, angular_speed_degree, clkwise):

    delay_obj= rospy.Rate(20)
    angular_speed=math.radians(angular_speed_degree)

    if clkwise:
        angular_speed= -abs(angular_speed)

    else:
        angular_speed= abs(angular_speed)

    velocity_message.angular.z = angular_speed
    velocity_publisher.publish(velocity_message)
    delay_obj.sleep()



def average_between_indices(ranges, i, j):
    ranges = [x for x in ranges if not math.isnan(x)]
    #print("Range length: "+ str(len(ranges)))
    slice_of_array_i= ranges[0: i+1]
    slice_of_array_j= ranges[j:]
    avg_left= sum(slice_of_array_i)/float(len(slice_of_array_i))
    avg_right= sum(slice_of_array_j)/float(len(slice_of_array_j))
    avg= (avg_left+avg_right)/2

    return avg, avg_left, avg_right

def average_sideways(ranges, i, j):
    ranges = [x for x in ranges if not math.isnan(x)]
    slice_of_array = ranges[i: j+1]
    return ( sum(slice_of_array) / float(len(slice_of_array)) )

if __name__ == '__main__':
    
    #init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)

    #subscribe to the topic /scan. 
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    time.sleep(1)

    #create a velocity publisher on topic /cmd_vel
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    

    global critical_distance, critical_left, critical_right

    initial_time= time.time()

    critical_min= 0.6
    critical_max= 2.5
    back_clearance_max= 0.40
    side_clearance= 0.30
    min_linear_speed= 0.28 #0.28
    min_ang_speed= 15
    max_reverse_speed= 0.18
    KP=0.09
    KP_ang=15.0
    

    while True:

        while ((critical_distance>=critical_min) and ((critical_left_corner>=side_clearance) and (critical_right_corner>=side_clearance))):

            
            linear_speed= KP*abs(critical_distance-critical_min)
        
            if (math.isinf(linear_speed)):
                linear_speed= KP*abs(critical_max-critical_min)

            if (linear_speed<=min_linear_speed):
                linear_speed=min_linear_speed

            move_straightline(velocity_publisher, linear_speed)
            
            print("Linear Speed: "+ str(linear_speed))

        
        if ((critical_left_corner<=side_clearance) or (critical_right_corner<=side_clearance)):
            if (critical_left_corner<=critical_right_corner):
                clkwise= True
            else:
                clkwise= False

            while ((critical_left_corner<=side_clearance) or (critical_right_corner<=side_clearance)):
                rotate(velocity_publisher, 10, clkwise)
                #reverse is initiated if the back is clear
                if (back_clearance>=back_clearance_max):
                    move_straightline(velocity_publisher, -max_reverse_speed)
                elif (back_clearance<back_clearance_max):
                    move_straightline(velocity_publisher, 0.0)
        
        rotate(velocity_publisher, 0.0, False)
        move_straightline(velocity_publisher, 0.0)

        

        if (critical_distance<=critical_max):
            if (critical_left_corner<=critical_right_corner):
                clkwise= True
            else:
                clkwise= False

            while (critical_distance<=critical_max):
    
                
                ang_speed= KP_ang*abs((critical_distance-critical_max))

                if (math.isinf(ang_speed)):
                    ang_speed= KP_ang*abs((critical_min-critical_max))

                if (ang_speed<=min_ang_speed):
                    ang_speed=min_ang_speed

                rotate(velocity_publisher, ang_speed, clkwise)
                print("Ang Speed: "+str(ang_speed))

        rotate(velocity_publisher, 0.0, False)
        move_straightline(velocity_publisher, 0.0)


        #if critical distance is cleared but any of the side corners are not cleared
        if (critical_distance>=critical_max and ((critical_left_corner<=side_clearance) or (critical_right_corner<=side_clearance))):
            
            if (critical_left_corner<=critical_right_corner):
                clkwise= True
            else:
                clkwise= False
            while ((critical_left_corner<=side_clearance) or (critical_right_corner<=side_clearance)):
                #print("Inside LOOP 3a!")
                rotate(velocity_publisher, 10, clkwise)
                if (back_clearance>=back_clearance_max):
                    move_straightline(velocity_publisher, -max_reverse_speed)
                elif (back_clearance<back_clearance_max):
                    move_straightline(velocity_publisher, 0.0)



        rotate(velocity_publisher, 0.0, False)
        move_straightline(velocity_publisher, 0.0)

    
        #run for 60 sec only
        delta_time= time.time() - initial_time

        if (delta_time>=60):
            break




   
        
        
