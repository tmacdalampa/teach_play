#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs import SetBoolRequest, SetBoolResponse

def callback(data):
    triggerPauseArmService = rospy.ServiceProxy('/pause_arm_service', SetBool)
    req = SetBoolRequest()
    laser_range = data.ranges
    res = any(i <= 1 for i in laser_range)
    if res == True:
        req.data = True
        #rospy.loginfo("something in")
    else:
        req.data = False
        #rospy.loginfo("safe")
    try: 
        res = triggerPauseArmService(req)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    

def listener():
    rospy.init_node('LaserScanSubscriber', anonymous=True)
    rospy.Subscriber("/octopoda/amr0/front_scan", LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()