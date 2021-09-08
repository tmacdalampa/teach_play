#!/usr/bin/env python
import sys
import time
import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, SetBool, SetBoolRequest, TriggerRequest, SetBoolResponse, TriggerResponse
from teach_play.srv import MotionPlanning, MotionPlanningRequest

class SelectMode(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['teach','play', 'failed', 'error', 'exit', 'straight', 'clear'])
		rospy.wait_for_service('/select_mode_service')
		rospy.wait_for_service('/go_straight_service')
		rospy.wait_for_service('/clear_pts_service')

		self.triggerSelectMode_service = rospy.ServiceProxy('/select_mode_service', SetBool)
		self.triggerGoStraight_service = rospy.ServiceProxy('/go_straight_service', Trigger)
		self.triggerClearPts_service = rospy.ServiceProxy('/clear_pts_service', Trigger)


	
	def execute(self, userdata):
		
		mode = raw_input("please input 'teach' or 'play' or 'exit' or 'straight' or 'clear':")
		req = SetBoolRequest()
		if mode == 'teach':
			req.data = True
			res = self.triggerSelectMode_service(req)
			if res.success == True:
				return 'teach'
			else:
				return 'faliled'

		elif mode == 'play':
			req.data = False
			res = self.triggerSelectMode_service(req)
			if res.success == True:
				return 'play'
			else:
				return 'faliled'
		elif mode == 'exit':
			rospy.signal_shutdown("shutdown for no reason.")
			return 'exit'
		elif mode == 'straight':
			res = self.triggerGoStraight_service(TriggerRequest())
			if res.success == True:
				return 'straight'
			else:
				return 'faliled'

		elif mode == 'clear':
			res = self.triggerClearPts_service(TriggerRequest())
			if res.success == True:
				return 'clear'
			else:
				return 'faliled'
		else:
			return 'error'
	
class RememberPoint(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['continue','done', 'failed'])
		rospy.wait_for_service('/remember_pts_service')
		self.triggerRememberPoint_service = rospy.ServiceProxy('/remember_pts_service', Trigger)
	
	def execute(self, userdata):
		pt = raw_input("please input 'p' when you want to remember this point and 'last' when you finish last point:")
		if pt == 'p':
			res = self.triggerRememberPoint_service(TriggerRequest())
			print('remember point succeed')
			return 'continue'
		elif pt == 'last':
			res = self.triggerRememberPoint_service(TriggerRequest())
			print('remember last point succeed')
			return 'done'
		else:
			print('enter wrong command, please try again')
			return 'failed'

class StartMotion(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeed','failed', 'continue'])
		rospy.wait_for_service('/play_pts_service')
		self.triggerStartMotion_service = rospy.ServiceProxy('/play_pts_service', MotionPlanning)
		self._motion_cnts = 1
		self._vel = 10
		self._motion_type = 'n'

	def execute(self, userdata):
		if (self._motion_cnts == 1):

			self._vel = input("please input maximum velocity percentage:")
			while(self._vel <= 0 or self._vel > 100):
				self._vel = input("please input maximum velocity percentage again:")
		
			req = MotionPlanningRequest()
			req.vel = self._vel

			self._motion_type = raw_input("please input motion type 'b' blending move or 'n' non blending move:")
			while(self._motion_type != 'b' and self._motion_type != 'n'):
				self._motion_type = input("please input motion type again:")
		
			if self._motion_type == 'b':
				req.type = True
			else:
				req.type = False

			#res = self.triggerStartMotion_service(req)
			#print(res)
			
		else:
			req = MotionPlanningRequest()
			req.vel = self._vel + 5*i
			req.type = False

		res = self.triggerStartMotion_service(req)
		print(res)
		self._motion_cnts = self._motion_cnts + 1
			

		if (res.success == True and self._motion_cnts > 3):
			return 'succeed'
		elif res.success == True:
			return 'continue'
		else:
			return 'failed'


def main():

	rospy.init_node('teach_play_state_machine')

	sm = smach.StateMachine(outcomes=['ended'])

	sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/IRA_Arm')



	with sm:
		smach.StateMachine.add('SelectMode', SelectMode(),
								transitions={'teach':'RememberPoint',
											'play':'StartMotion',
											'failed':'ended',
											'error':'SelectMode',
											'exit':'ended', 
											'straight':'SelectMode',
											'clear':'SelectMode'})

		smach.StateMachine.add('RememberPoint', RememberPoint(),
								transitions={'continue':'RememberPoint',
											'done':'SelectMode', 
											'failed':'ended'})
		
		smach.StateMachine.add('StartMotion', StartMotion(),
								transitions={'succeed':'SelectMode',
											'failed':'ended',
											'continue':'StartMotion'})


		
		
	sis.start()
	# Execute SMACH plan
	outcome = sm.execute()
    
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()