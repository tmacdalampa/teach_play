#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Float32
from teach_play.srv import LaserManager, LaserManagerRequest, LaserManagerResponse

triggerPauseArmService = None

class LaserScanManager():
    
    def __init__(self, distance):

        self._distance = distance
        self._value = 'null'
        self._triggerPauseArmService = rospy.ServiceProxy('/laser_manager_service', LaserManager)
        self._req = LaserManagerRequest()
    def callback(self, data):        

        """
        if data.data>distance[1]:
            self._req.vel_factor = 1
            state = 'safe'
            
        elif (data.data >= distance[0] and data.data <= distance[1]):
            self._req.vel_factor = 0.5
            state = 'warn'
        else:
            self._req.vel_factor = 0
            state = 'stop'
            
        if (state != self._value):
            try:
                res = self._triggerPauseArmService(self._req)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            rospy.loginfo(state)

        self._value = state
        """

        
        req = LaserManagerRequest()
        laser_range = data.ranges
        
        res_stop = any(i < self._distance[0] for i in laser_range)
        res_warn = any(i < self._distance[1] for i in laser_range)
        
        if (res_stop == True) :
            state = 'stop'
            req.vel_factor = 0
        elif(res_warn == True and res_stop == False):
            state = 'warn'
            req.vel_factor = 0.1
        else:
            state = 'safe'
            req.vel_factor = 1
         
        rospy.loginfo(state)
        
        if (state != self._value):
            rospy.loginfo(state)
            
            try:
                res = self._triggerPauseArmService(req)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        
        self._value = state
        
    
    def listener(self):
        
        rospy.init_node('LaserScanManager', anonymous=True)
        rospy.Subscriber("/octopoda/amr0/front_scan", LaserScan, self.callback)
        #rospy.Subscriber("/octopoda/amr0/front_scan", Float32, self.callback)
        
        
        rospy.spin()

if __name__ == '__main__':
    
    distance = [0.5, 1]
    mymanager = LaserScanManager(distance)
    
    mymanager.listener()