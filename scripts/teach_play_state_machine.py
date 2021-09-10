#!/usr/bin/env python
import sys
import time
import roslib
import rospy
import smach
import smach_ros
import actionlib
from std_srvs.srv import Trigger, SetBool, SetBoolRequest, TriggerRequest, SetBoolResponse, TriggerResponse
from teach_play.srv import MotionPlanning, MotionPlanningRequest
from teach_play.msg import MoveLinearAbsAction, MoveLinearAbsGoal, MoveLinearAbsResult

class SelectMode(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['teach','play', 'failed', 'error', 'exit', 'straight', 'clear'])
		rospy.wait_for_service('/select_mode_service')
		rospy.wait_for_service('/go_straight_service')
		rospy.wait_for_service('/clear_pts_service')

		self.triggerSelectMode_service = rospy.ServiceProxy('/select_mode_service', SetBool)
		self.triggerGoStraight_service = rospy.ServiceProxy('/go_straight_service', Trigger)
		self.triggerClearPts_service = rospy.ServiceProxy('/clear_pts_service', Trigger)
		print('initial state SelectMode')

	
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
		print('initial state Remember Point')

	
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
		#rospy.wait_for_service('/play_pts_service')
		#self.triggerStartMotion_service = rospy.ServiceProxy('/play_pts_service', MotionPlanning)

		self.client = actionlib.SimpleActionClient('/move_linear_abs', MoveLinearAbsAction)
		self.client.wait_for_server()
		print('initial state StartMotion')

	
	def execute(self, userdata):
		goal = MoveLinearAbsGoal()
		goal.vel = input("please input maximum velocity percentage:")
		
		while(goal.vel <= 0 or goal.vel > 100):
			goal.vel = input("please input maximum velocity percentage again:")

		motion_type = raw_input("please input motion type 'b' blending move or 'n' non blending move:")
		
		while(motion_type != 'b' and motion_type != 'n'):
			motion_type = input("please input motion type again:")
		
		if motion_type == 'b':
			goal.type = True
		else:
			goal.type = False

		self.client.send_goal(goal)
		self.client.wait_for_result()
		res = self.client.get_result()

		if res.success == True:
			return 'succeed'
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
											'failed':'ended'})


		
		
	sis.start()
	# Execute SMACH plan
	outcome = sm.execute()
    
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()