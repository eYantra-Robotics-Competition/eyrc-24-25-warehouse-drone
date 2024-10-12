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
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray, Pose
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node

SAMPLE_TIME=0.060
ARM_RC_VALUE = 1500
DISARM_RC_VALUE = 1000

class PIDController:
	def __init__(self, sample_time, max_value, min_value, baseline, Kp=0.0, Ki=0.0, Kd=0.0):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.sample_time = sample_time
		self.baseline_out = baseline
		self.max_out = max_value
		self.min_out = min_value
		self.prev_error = 0.0
		self.error_sum = 0.0

	def compute(self, setpoint:float, measured_value:float, saturate=True)->float:
		error = measured_value - setpoint
		print(f"[PIDController] error {error}")
		self.error_sum += error * self.sample_time
		
		P_term = self.Kp * error
		I_term = self.Ki * self.error_sum
		D_term = self.Kd * (error - self.prev_error) / self.sample_time

		output = self.baseline_out + P_term + I_term + D_term
		
		if saturate:
			output = max(min(output, self.max_out), self.min_out)

		self.prev_error = error
		return output


class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_pose = Pose()
		self.drone_position = [0.0, 0.0, 0.0]
		self.drone_position_valid = False

		# [x_setpoint, y_setpoint, z_setpoint]
		self.set_pose = Pose()
		self.set_pose.position.x = 2.0
		self.set_pose.position.y = 2.0
		self.set_pose.position.z = 19.0

		self.setpoint = [2, 2, 19]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = ARM_RC_VALUE
		self.cmd.rc_pitch = ARM_RC_VALUE
		self.cmd.rc_yaw = ARM_RC_VALUE
		self.cmd.rc_throttle = ARM_RC_VALUE

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		
		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_error = [0.0, 0.0, 0.0]  # previous errors for [roll, pitch, throttle]
		self.error_sum = [0.0, 0.0, 0.0]  # cumulative errors for [roll, pitch, throttle]

		self.max_values = [2000, 2000, 2000]  # Maximum values for [roll, pitch, throttle]
		self.min_values = [1000, 1000, 1000]  # Minimum values for [roll, pitch, throttle]

		self.sample_time = SAMPLE_TIME
		
		self.altitud_controller = PIDController(sample_time=SAMPLE_TIME, max_value=2000, min_value=1000, baseline=1500)


		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.set_throttle_pid_callback, 1)

		self.arm()  # ARMING THE DRONE
		self.timer = self.create_timer(self.sample_time, self.pid)

		self.get_logger().info(f"{self.get_name()} initialized")


	def disarm(self):
		self.cmd.rc_roll = DISARM_RC_VALUE
		self.cmd.rc_yaw = DISARM_RC_VALUE
		self.cmd.rc_pitch = DISARM_RC_VALUE
		self.cmd.rc_throttle = DISARM_RC_VALUE
		self.cmd.rc_aux4 = DISARM_RC_VALUE
		self.command_pub.publish(self.cmd)
		self.get_logger().info(f"{self.get_name()} disarmed")

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command
		self.get_logger().info(f"{self.get_name()} armed")

	def whycon_callback(self, pose_array_msg:PoseArray):
		self.drone_position[0] = pose_array_msg.poses[0].position.x  
		self.drone_position[1] = pose_array_msg.poses[0].position.y
		self.drone_position[2] = pose_array_msg.poses[0].position.z

		if self.drone_position != [0.0, 0.0, 0.0]:
			self.drone_position_valid = True
		
		self.drone_pose = pose_array_msg.poses[0]

	def set_throttle_pid_callback(self, pid_msg:PIDTune):
		self.Kp[2] = pid_msg.kp * 0.01  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = pid_msg.ki * 0.0001
		self.Kd[2] = pid_msg.kd * 0.1

		self.altitud_controller.Kp = pid_msg.kp
		self.altitud_controller.Ki = pid_msg.ki
		self.altitud_controller.Kd = pid_msg.kd

	def print_pid_gains(self):
		self.get_logger().info(f"Kp: {self.altitud_controller.Kp}, Ki: {self.altitud_controller.Ki}, Kd: {self.altitud_controller.Kd}")

	def pid(self):
		if not self.drone_position_valid:
			self.get_logger().info("Waiting for valid drone position")
			return

		self.get_logger().info(f'drone position z {self.drone_pose.position.z}')
		throttle_cmd = self.altitud_controller.compute(self.set_pose.position.z, self.drone_pose.position.z, saturate=True)
		self.print_pid_gains()


		self.cmd.rc_throttle = int(throttle_cmd)

		# Step 6: Publish command after all adjustments
		self.get_logger().info(f'Publishing command {throttle_cmd}')
		self.command_pub.publish(self.cmd)

def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	rclpy.spin(swift_pico)
	swift_pico.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
