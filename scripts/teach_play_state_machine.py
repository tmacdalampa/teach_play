#!/usr/bin/env python
import sys
import time
import roslib
import rospy
import smach
import smach_ros
import actionlib
from std_srvs.srv import Trigger, SetBool, SetBoolRequest, TriggerRequest, SetBoolResponse, TriggerResponse
from teach_play.srv import MotionPlanning, MotionPlanningRequest, Decode, DecodeRequest
from teach_play.msg import MoveLinearAbsAction, MoveLinearAbsGoal, MoveLinearAbsResult

class SelectMode(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['teach','play', 'failed', 'error', 'exit', 'clear'],
							input_keys=['selectmode_input'],
							output_keys=['selectmode_out'])
		
		rospy.wait_for_service('/select_mode_service')
		rospy.wait_for_service('/clear_pts_service')

		self.triggerSelectMode_service = rospy.ServiceProxy('/select_mode_service', SetBool)
		self.triggerClearPts_service = rospy.ServiceProxy('/clear_pts_service', Trigger)
	
	def execute(self, userdata):
		if userdata.selectmode_input == 'jog':
			req = SetBoolRequest()
			req.data = True
			res = self.triggerSelectMode_service(req)
			if res.success == True:
				return 'teach'
			else:
				return 'faliled'
		else:		
			mode = raw_input("please input 'teach' or 'play' or 'exit' or 'clear':")
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
				userdata.selectmode_out = 'play'
				if res.success == True:
					return 'play'
				else:
					return 'faliled'
			elif mode == 'exit':
				rospy.signal_shutdown("shutdown for no reason.")
				return 'exit'

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
		smach.State.__init__(self, 
							outcomes=['continue','done', 'jog','failed'],
							output_keys=['rememberpoint_out'])
		rospy.wait_for_service('/remember_pts_service')
		self.triggerRememberPoint_service = rospy.ServiceProxy('/remember_pts_service', Trigger)
		
	
	def execute(self, userdata):

		pt = raw_input("please input 'p' when you want to remember this point and 'last' when you finish last point or 'jog' to move a little bit:")
		if pt == 'p':
			res = self.triggerRememberPoint_service(TriggerRequest())
			print('remember point succeed')
			return 'continue'
		
		elif pt == 'last':
			res = self.triggerRememberPoint_service(TriggerRequest())
			userdata.rememberpoint_out = 'play'
			print('remember last point succeed')
			return 'done'
		
		elif pt == 'jog':
			return 'jog'
		else:
			print('enter wrong command, please try again')
			return 'continue'

class StartMotion(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['succeed','failed'],
							input_keys = ['startmotion_input'],
							output_keys = ['startmotion_out'])


		self.client = actionlib.SimpleActionClient('/move_linear_abs', MoveLinearAbsAction)
		self.client.wait_for_server()

	def execute(self, userdata):
		goal = MoveLinearAbsGoal()
		if userdata.startmotion_input == 'play':

			goal.vel = input("please input maximum velocity percentage:")
			
			while(goal.vel <= 0 or goal.vel > 100):
				goal.vel = input("please input maximum velocity percentage again:")

			motion_type = raw_input("please input motion type 'p' move points or 'o' move straight:")
			
			while(motion_type != 'p' and motion_type != 'o'):
				motion_type = input("please input motion type again:")
			
			if motion_type == 'o':
				goal.type = 1 #go straight
			else:
				goal.type = 0 #play points

			self.client.send_goal(goal)
			self.client.wait_for_result()
			res = self.client.get_result()
			userdata.startmotion_out = userdata.startmotion_input
		else:
			goal.vel = 5
			goal.type = 2 #jog
			self.client.send_goal(goal)
			self.client.wait_for_result()
			res = self.client.get_result()
			userdata.startmotion_out = userdata.startmotion_input

		if res.success == True:
			return 'succeed'
		else:
			return 'failed'

class JogGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['succeed','failed'],
							output_keys=['joggoal_out'])
		
		self.triggerJogGoal_service = rospy.ServiceProxy('/jog_goal_service', Decode)
	def execute(self, userdata):
		req = DecodeRequest()
		userdata.joggoal_out = 'jog'
		script_type = raw_input("please input 'AR', 'AA' , 'MR', 'MA':")
		while(script_type != 'MR' and script_type != 'MA' and script_type != 'AA' and script_type != 'AR'):
			script_type = input("please input 'AR', 'AA' , 'MR', 'MA' again:")
		
		if script_type == 'AA':
			req.type = 0
		elif script_type == 'AR':
			req.type = 1
		elif script_type == 'MA':
			req.type = 2
		else:
			req.type = 3

		displacement = raw_input("please input 6 arguments:")
		user_list = displacement.split()
		
		while(len(user_list) != 6):
			displacement = raw_input("please input 6 arguments:")
			user_list = displacement.split()
		
		for i in range(len(user_list)):		
			req.position.append(float(user_list[i]))


		res = self.triggerJogGoal_service(req)
		del req.position[:]
		if res.success == True:
			return 'succeed'
		else:
			return 'failed'
			


def main():

	rospy.init_node('teach_play_state_machine')

	sm = smach.StateMachine(outcomes=['ended'])

	sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/IRA_Arm')
	sm.userdata.start_motion_type = 'play'

	with sm:
		smach.StateMachine.add('SelectMode', SelectMode(),
								transitions={'teach':'RememberPoint',
											'play':'StartMotion',
											'failed':'ended',
											'error':'SelectMode',
											'exit':'ended', 
											'clear':'SelectMode'},
								remapping={'selectmode_input':'start_motion_type',
											'selectmode_out':'start_motion_type'})

		smach.StateMachine.add('RememberPoint', RememberPoint(),
								transitions={'continue':'RememberPoint',
											'done':'SelectMode', 
											'failed':'ended',
											'jog':'JogGoal'},
								remapping={'rememberpoint_out':'start_motion_type'})
		
		smach.StateMachine.add('StartMotion', StartMotion(),
								transitions={'succeed':'SelectMode',
											'failed':'ended'},
								remapping={'startmotion_input':'start_motion_type',
											'startmotion_out':'selectmode_input'})

		smach.StateMachine.add('JogGoal', JogGoal(),
								transitions={'succeed':'StartMotion',
											'failed':'ended'},
								remapping={'joggoal_out':'start_motion_type'})
		
		
	sis.start()
	# Execute SMACH plan
	outcome = sm.execute()
    
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()