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



class MasterNode(object):
    count = 0 
    def __init__(self):
        self.motor_pub = rospy.Publisher("motor_cmd", String, queue_size=5)
        self.ping_sub = rospy.Subscriber("ping_sensor", Range, self.callback) #continously calls cllback when data is recived
        self.cur_range  # making this global so any update anyware gets immediately reflected 
        self.time_array = []
        self.d_array = []
        self.min_distance = 5
        self.start_time_count = 0
        self.aligndistance = 0
        print("Initiated Node")

    def callback(self, data):
        self.cur_range = data.range
        self.check_range()               #Doubt will it execute every 500ms or wait till 2 secs
        # self.motor_pub.publish(cur_cmd)
        # rospy.loginfo("Bot is moving - {0}".format(cur_cmd))

    def publish_cmd(self, motor_cmd):
        self.motor_pub.publish(motor_cmd)
        rospy.loginfo("Bot is moving - {0}".format(motor_cmd))  # publish
        return

    def check_range(self):
         # direction array
     
        motor_cmd = "F"
        
        if self.cur_range < self.min_distance and self.count == 0:  # 1ST major turn # make this run only once***
            motor_cmd = "R"
            self.d_array.append("R")
            
            self.count = 1
            self.publish_cmd(motor_cmd)  # publish
            self.check_range_2()
            #motor_cmd = "F"                             #not needed i guess
            # self.motor_pub.publish()
            # rospy.loginfo("Bot is moving - {0}".format(motor_cmd))
        elif self.count == 0 :
            motor_cmd = "F"
            self.publish_cmd(motor_cmd)
       
       
        
        elif(motor_cmd == "F" and self.count>=1):  # will be left in T
            self.check_range_2(motor_cmd)
        
        
    def check_range_2(self):
            self.start_time_count = time.time()
            motor_cmd = self.min_turn()    # call min_turn
            self.publish_cmd(motor_cmd)  # publish
            while(motor_cmd!='F'):
                motor_cmd=self.minturn()
                self.publish_cmd(motor_cmd)
            self.time_array.append(self.start_time_count - time.time())





    def halt_bot(self):
        rospy.loginfo("Bot is coming to halt!")
        self.motor_pub.publish("X")
    

    def min_turn(self):                 # dire -direction
        max_time = 2                        # adjust this time in seconds to increse or decrese number of min turns thus moving faster
        start_time = time.time()
        while((time.time()-start_time) < max_time):     # to keep moving forward for fixed time
            self.motor_pub.publish('F')
            rospy.loginfo("Bot is moving forward for 2 secs- {0}".format('F'))
       
        motor_cmd = "L"
        self.publish_cmd(motor_cmd)  # publish
        if(self.cur_range <= self.min_distance ):
            return "R"             # revert back cause min distance> cur_range dictonary of opp
        else:
            self.d_array.append(motor_cmd)  # add todirection array
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
