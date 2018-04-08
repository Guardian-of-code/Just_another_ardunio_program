# -*- coding: utf-8 -*-
"""
Created on Sun Apr  1 01:06:20 2018

@author: sg
"""
import time
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range

d = {'L' : 'R','R' : 'L'}   # dictionary so that for min_turn it turns in opposite direction
global cur_range  # making this global so any update anyware gets immediately reflected 


class MasterNode(object):
    def __init__(self):
        self.motor_pub = rospy.Publisher("motor_cmd", String, queue_size=5)
        self.ping_sub = rospy.Subscriber("ping_sensor", Range, self.callback) #continously calls cllback when data is recived
        self.min_distance = 5
        self.start_time_count = 0
        self.aligndistance = 0
        print("Initiated Node")

    def callback(self, data):
        global cur_range
        cur_range = data.range
        # self.check_range(cur_range)
        # self.motor_pub.publish(cur_cmd)
        # rospy.loginfo("Bot is moving - {0}".format(cur_cmd))

    def publish_cmd(self, motor_cmd):
        self.motor_pub.publish(motor_cmd)
        rospy.loginfo("Bot is moving - {0}".format(motor_cmd))  # publish
        return

    def check_range(self):
        global d_array
        d_array = []
        global time_array
        time_array = []
        motor_cmd = "F"
        if cur_range < self.min_distance:  # 1ST major turn
            motor_cmd = "R"
            array.append("R")
            self.publish_cmd(motor_cmd)  # publish
            #motor_cmd = "F"                             #not needed i guess
            #self.motor_pub.publish()
            #rospy.loginfo("Bot is moving - {0}".format(motor_cmd))
        else:
            motor_cmd = "F"
        if(motor_cmd == "R"):
            self.start_time_count = time.time()
            motor_cmd = self.min_turn()    # call min_turn
            self.publish_cmd(motor_cmd)  # publish
        
            if(motor_cmd == "F"):
                time_array.append(self.start_time_count - time.time())
                
                     
            else:
                # now to loop it
                
                
                
        
       

    def halt_bot(self):
        rospy.loginfo("Bot is coming to halt!")
        self.motor_pub.publish("X")
    

    def min_turn(self):                 # dire -direction
        max_time = 2                        # adjust this time in seconds to increse or decrese number of min turns thus moving faster
        start_time = time.time()
        while(time.time()-start_time) < max_time:     # to keep moving forward for fixed time
            self.motor_pub.publish('F')
            rospy.loginfo("Bot is moving - {0}".format('F'))
        global d_array
        motor_cmd = "L"
        self.publish_cmd(motor_cmd)  # publish
        if(cur_range <= self.min_distance ):
            return "R"             # revert back cause min distance> cur_range dictonary of opp
        else:
            d_array.append(motor_cmd)
            return "F"                     # no obstacal move forward
            
        
	


def main(args):
    rospy.init_node('master_node', anonymous=False)
    mn = MasterNode()
    try: 
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        print("Shutting down for {0}".format(e))    
    rospy.on_shutdown(mn.halt_bot())


if __name__ == '__main__':
    main(sys.argv)
