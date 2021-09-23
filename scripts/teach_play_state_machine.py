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
							output_keys = ['selectmode_out'])
		rospy.wait_for_service('/select_mode_service')
		rospy.wait_for_service('/clear_pts_service')

		self.triggerSelectMode_service = rospy.ServiceProxy('/select_mode_service', SetBool)
		self.triggerClearPts_service = rospy.ServiceProxy('/clear_pts_service', Trigger)

	
	def execute(self, userdata):
		
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
							output_keys = ['rememberpoint_out'])
		rospy.wait_for_service('/remember_pts_service')
		self.triggerRememberPoint_service = rospy.ServiceProxy('/remember_pts_service', Decode)
		self.req = DecodeRequest()
	
	def execute(self, userdata):
		pt = raw_input("please input 'p' when you want to remember this point and 'last' when you finish last point or 'jog' to move a little bit:")
		if pt == 'p':
			self.req = 0
			res = self.triggerRememberPoint_service(req)
			print('remember point succeed')
			return 'continue'
		
		elif pt == 'last':
			self.req = 0
			res = self.triggerRememberPoint_service(req)
			print('remember last point succeed')
			return 'done'
		
		elif pt == 'jog':
			userdata.rememberpoint_out = 'jog'
			script_type = raw_input("please input 'AR', 'AA' , 'MR', 'MA'")
			
			while(script_type != 'MR' or script_type != 'MA' or script_type != 'AA' or script_type != 'AR'):
				script_type = input("please input maximum velocity percentage again:")

			displacement = raw_input("please input 6 arguments")
			user_list = displacement.split()
			
			while(len(user_list) != 6):
				displacement = raw_input("please input 6 arguments")
				user_list = displacement.split()
			
			for i in range(len(user_list)):
    			# convert each item to int type
    			user_list[i] = float(user_list[i])
    			req.position[i] = user_list[i]
			
			res = self.triggerRememberPoint_service(req)
			userdata.rememberpoint_out = 'jog'
			return 'jog'

		else:
			print('enter wrong command, please try again')
			return 'failed'

class StartMotion(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['succeed','failed', 'back'],
							input_keys = ['startmotion_input'])


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

			if res.success == True:
				return 'succeed'
			else:
				return 'failed'
			
		else:
			goal.vel = 5
			goal.type = 2 #jog
			self.client.send_goal(goal)
			self.client.wait_for_result()
			res = self.client.get_result()

			if res.success == True:
				return 'back'
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
											'clear':'SelectMode'},
								remapping={'selectmode_out':'start_motion_type'})

		smach.StateMachine.add('RememberPoint', RememberPoint(),
								transitions={'continue':'RememberPoint',
											'done':'SelectMode', 
											'failed':'ended',
											'jog':'StartMotion'},
								remapping={'rememberpoint_out':'start_motion_type'})
		
		smach.StateMachine.add('StartMotion', StartMotion(),
								transitions={'succeed':'SelectMode',
											'failed':'ended',
											'back', 'RememberPoint'},
								remapping={'startmotion_input':'start_motion_type'})


		
		
	sis.start()
	# Execute SMACH plan
	outcome = sm.execute()
    
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()