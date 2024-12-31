#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			/throttle_pid
						/pitch_pid
						/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries
import scipy.signal
import numpy as np
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node

MIN_ROLL = 1200
BASE_ROLL = 1500
MAX_ROLL = 1700
SUM_ERROR_ROLL_LIMIT = 5000

MIN_PITCH = 1200
BASE_PITCH = 1500
MAX_PITCH = 1700
SUM_ERROR_PITCH_LIMIT = 5000

MIN_THROTTLE = 1250
BASE_THROTTLE = 1500
MAX_THROTTLE = 2000
SUM_ERROR_THROTTLE_LIMIT = 5000

CMD = [[], [], []]


class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0, 0.0, 0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [3, -3, 27]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = RCMessage()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		#-----------------------Add other required variables for pid here ----------------------------------------------









		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		self.sample_time = 0.060  # in seconds
		
		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(RCMessage, '/drone/rc_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)

		#------------------------Add other ROS Subscribers here-----------------------------------------------------
	

		#arm/disarm service client
		self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Arming service not available, waiting again,,,,')
		self.req = CommandBool.Request()

		future = self.send_request() # ARMING THE DRONE
		rclpy.spin_until_future_complete(self, future)
		response = future.result()
		self.get_logger().info(response.data)

		# Create a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)


    #drone arm/disarm function 
	def send_request(self):
		self.req.value = True
		return self.cli.call_async(self.req)


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self, msg):
		self.drone_position[0] = msg.poses[0].position.x 
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------



	
		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki * 0.008
		self.Kd[2] = alt.kd * 0.6

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	#----------------------------------------------------------------------------------------------------------------------

	def publish_filtered_data(self, roll, pitch, throttle):

		self.cmd.rc_throttle = int(throttle)
		self.cmd.rc_roll = int(roll)
		self.cmd.rc_pitch = int(pitch)
		self.cmd.rc_yaw = int(1500)


		# BUTTERWORTH FILTER low pass filter
		span = 15
		for index, val in enumerate([roll, pitch, throttle]):
			CMD[index].append(val)
			if len(CMD[index]) == span:
				CMD[index].pop(0)
			if len(CMD[index]) != span-1:
				return
			order = 3 # determining order 
			fs = 30 # to keep in order same as hz topic runs
			fc = 4 
			nyq = 0.5 * fs
			wc = fc / nyq
			b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
			filtered_signal = scipy.signal.lfilter(b, a, CMD[index])
			if index == 0:
				self.cmd.rc_roll = int(filtered_signal[-1])
			elif index == 1:
				self.cmd.rc_pitch = int(filtered_signal[-1])
			elif index == 2:
				self.cmd.rc_throttle = int(filtered_signal[-1])

			if self.cmd.rc_roll > MAX_ROLL:     #checking range i.e. bet 1000 and 2000
				self.cmd.rc_roll = MAX_ROLL
			elif self.cmd.rc_roll < MIN_ROLL:
				self.cmd.rc_roll = MIN_ROLL

			# Similarly add bounds for pitch yaw and throttle 
			if self.cmd.rc_throttle > MAX_THROTTLE:
				self.cmd.rc_throttle = MAX_THROTTLE
			elif self.cmd.rc_throttle < MIN_THROTTLE:
				self.cmd.rc_throttle = MIN_THROTTLE

			if self.cmd.rc_pitch > MAX_PITCH:
				self.cmd.rc_pitch = MAX_PITCH
			elif self.cmd.rc_pitch < MIN_PITCH:
				self.cmd.rc_pitch = MIN_PITCH

		self.command_pub.publish(self.cmd)

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#   3. Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)
	#	4. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	5. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	8. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	9. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	10. Add error_sum


		




		# output values of PID
		self.roll_out = 
        
		self.pitch_out = 

		self.throttle_out = 
	#------------------------------------------------------------------------------------------------------------------------
		self.publish_filtered_data(roll = self.roll_out,pitch = self.pitch_out,throttle = self.throttle_out)
		# calculate throttle error, pitch error and roll error, then publish it accordingly
		self.pid_error_pub.publish(self.pid_error)



def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()

	try:
		rclpy.spin(swift_pico)
	except KeyboardInterrupt:
		swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
	finally:
		swift_pico.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
