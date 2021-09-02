#!/usr/bin/env python
import rospy
from teach_play.srv import SpeedOverride, SpeedOverrideRequest, SpeedOverrideResponse


def ServerCallback(req):
    res = SpeedOverrideResponse()
    if (req.vel_factor == 0):
    	res.message = 'set vel factor 0'
    elif(req.vel_factor == 0.5):
    	res.message = 'set vel factor 0.5'
    else:
    	res.message = 'set vel factor 1'
    res.success = True
    print(res)
    return res

def SpeedOverrideServer():
    rospy.init_node('test_speed_override_server')
    s = rospy.Service('/speed_override_service', SpeedOverride, ServerCallback)
    print("Server Ready")
    rospy.spin()

if __name__ == "__main__":
    SpeedOverrideServer()