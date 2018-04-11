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
        self.start_time = 0
        self.max_time = 6
        self.d_array = []
        self.time_counter=0
        self.time_array=[0,0,0]
        self.temp_cmd = "Random"
        print("Initiated Node")

    def callback(self, data):
        cur_range = data.range
        cur_cmd = self.check_range(cur_range)
        self.motor_pub.publish(cur_cmd)
        rospy.loginfo("Bot is moving - {0}".format(cur_cmd))

    def check_range(self, distance):
        motor_cmd = "F"

        if self.temp_cmd == "L":
            motor_cmd = self.check1(distance)
            if motor_cmd=="R":
                self.time_counter+=6
                self.start_time = time.time()
            self.temp_cmd = "Random"

        elif distance < self.min_distance:
            self.start_time = time.time()
            motor_cmd = "R"
            self.d_array.append(motor_cmd)

            # determine how long to turn 90deg                        
        else:
            motor_cmd = "F"
            t = time.time()-self.start_time
            if(t >=self.max_time() and self.start_time!=0): #self.starttime probably not needed
                motor_cmd ="L"
                self.time_counter=6
                self.temp_cmd = motor_cmd
            if(self.time_array[0]<=(time.time()-self.start_time and self.time_array[1]!=0):
                self.start_time = 0
                self.max_time = 6
                self.d_array = []
                self.time_counter=0
                self.time_array=[0,0,0]
                self.temp_cmd = "Random"
                motor_cmd="R"
            
            
        return motor_cmd

    def halt_bot(self):
        rospy.loginfo("Bot is coming to halt!")
        self.motor_pub.publish("X")
    
    def check1(self,distance):
        #motor_cmd = "L"
        #self.motor_pub.publish(motor_cmd)
        #rospy.loginfo("Bot is moving - {0}".format(motor_cmd))  ## done
        # publish
        if(distance <= self.min_distance ):
            return "R"             # revert back cause min distance> cur_range dictonary of opp
        else:
            self.d_array.append("L")  # add todirection array
            self.time_array.append(self.time_counter)
            self.start_time = time.time()
            return "F"  
        
        

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


