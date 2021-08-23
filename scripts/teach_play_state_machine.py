#!/usr/bin/env python
import time
import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, SetBool, SetBoolRequest, TriggerRequest, SetBoolResponse, TriggerResponse
from teach_play.srv import MotionPlanning, MotionPlanningRequest

class SelectMode(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['teach','play', 'failed', 'error'])
		rospy.wait_for_service('/select_mode_service')
		self.triggerSelectMode_service = rospy.ServiceProxy('/select_mode_service', SetBool)
	
	def execute(self, userdata):
		
		mode = raw_input("please input 'teach' or 'play':")
		if mode == 'teach':
			req.data = True
		elif mode == 'play':
			req.data = False
		else:
			return 'error'

		res = self.triggerSelectMode_service(SetBoolRequest())
		
		if req.data == True and res.success == True:
			return 'teach'
		elif req.data == False and res.success == True:
			return 'play'
		else:
		  return 'failed'
	
class RememberPoint(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['continue','done', 'failed'])
		rospy.wait_for_service('/remember_pt_service')
		self.triggerRememberPoint_service = rospy.ServiceProxy('/remember_pt_service', Trigger)
	
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
		smach.State.__init__(self, outcomes=['succeed','failed'])
		rospy.wait_for_service('/start_play_service')
		self.triggerStartMotion_service = rospy.ServiceProxy('/start_play_service', MotionPlanning)
	
	def execute(self, userdata):
		vel = input("please input maximum velocity percentage:")
		
		while(vel <= 0 or vel > 100):
			vel = input("please input maximum velocity percentage again:")
		
		res = self.triggerStartMotion_service(MotionPlanningRequest())
		if res == True:
			return 'succeed'
		else:
			return 'failed'


def main():

	rospy.init_node('teach_play_state_machine')

	sm = smach.StateMachine(outcomes=['aborted'])

	sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/IRA_arm')



	with sm:
		smach.StateMachine.add('SelectMode', SelectMode(),
								transitions={'teach':'RememberPoint',
											'play':'StartMotion',
											'failed':'aborted',
											'error':'SelectMode'})

		smach.StateMachine.add('RememberPoint', RememberPoint(),
								transitions={'continue':'RememberPoint',
														'done':'SelectMode', 
														'failed':'aborted'})
		
		smach.StateMachine.add('StartMotion', StartMotion(),
								transitions={'succeed':'SelectMode',
														'failed':'aborted'})


		
		
	sis.start()
	# Execute SMACH plan
	outcome = sm.execute()
    
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()