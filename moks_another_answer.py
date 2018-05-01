# -*- coding: utf-8 -*-
"""
Team A
"""

import sys
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import Range


class MasterNode(object):
    def __init__(self):
        self.motor_pub = rospy.Publisher("motor_cmd", String, queue_size=5)
        self.ping_sub = rospy.Subscriber("ping_sensor", Range, self.callback)
        self.min_distance = 5
        self.timearray = [0, 0, 0]
        self.turn_delay = 3.5
        self.max_time = 5
        self.start_time = 0
        self.temp_cmd = "not left"
        self.start_time1 = 0
        self.timecounter = 0
        print("Initiated Node")

    def callback(self, data):
        cur_range = data.range
        cur_cmd = self.check_range(cur_range)
        if (cur_cmd == 'F'):
            self.motor_pub.publish(cur_cmd)
            rospy.loginfo("Bot is moving - {0}".format(cur_cmd))
        elif(cur_cmd == 'R'or cur_cmd == 'L'):
            self.motor_pub.publish('X')
            rospy.loginfo("Bot is moving - {0}".format(cur_cmd))
            self.motor_pub.publish(cur_cmd)
            rospy.loginfo("Bot is moving - {0}".format(cur_cmd))
            time.sleep(self.turn_delay)

    def check_range(self, distance):
        motor_cmd = "F"
        if self.temp_cmd == "L" and distance >= self.min_distance:
            t = time.time() - self.start_time1
            if(t >= self.max_time()):
                motor_cmd = "L"
                self.timearray.append(self.timecounter)
                self.temp_cmd == 'not Left'
                if(self.timearray[0] != 0 and self.timearray[1] != 0):
                    while(time.time-self.start_time1 <= self.timearray[0]):
                        self.motor_pub.publish('F')
                    motor_cmd = "R"
                    self.reset_values()

        elif distance <= self.min_distance:
            self.start_time = (self.turn_delay + time.time())
            motor_cmd = "R"

        else:
            motor_cmd = "F"
            t = time.time() - self.start_time
            if(t >= self.max_time() and self.start_time != 0): #self.starttime probably not needed	             if(t >=self.max_time() and self.start_time!=0): #self.starttime probably not needed
                motor_cmd = "L"
                self.time_counter += 12
                self.temp_cmd = motor_cmd
                self.start_time1 = (self.turn_delay+time.time())
                
        return motor_cmd

    def halt_bot(self):
        rospy.loginfo("Bot is coming to halt!")
        self.motor_pub.publish("X")
    def reset_values(self):
        self.timearray = [0, 0, 0]
        self.turn_delay = 3.5
        self.max_time = 5
        self.start_time = 0
        self.temp_cmd = "not left"
        self.start_time1 = 0
        self.timecounter = 0
        
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

