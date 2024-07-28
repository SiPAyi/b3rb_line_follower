# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import math

from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan


QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

# THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_VERTICAL = 0.25 # here i changed
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25


class LineFollower(Node):
	""" Initializes line follower node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('line_follower')

		# Subscription for edge vectors.
		self.subscription_vectors = self.create_subscription(
			EdgeVectors,
			'/edge_vectors',
			self.edge_vectors_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for joy (for moving the rover in manual mode).
		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Subscription for traffic status.
		self.subscription_traffic = self.create_subscription(
			TrafficStatus,
			'/traffic_status',
			self.traffic_status_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for LIDAR data.
		self.subscription_lidar = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			QOS_PROFILE_DEFAULT)

		self.traffic_status = TrafficStatus()

		self.obstacle_detected = False

		self.ramp_detected = False
		
		### edited by saik
		# for speed and steering angle
		self.current_speed = 0.0
		self.current_turn = 0.0
		self.left_turn_indication = 0.0
		self.right_turn_indication = 0.0 
		self.vector1_slope = 0.0 # left side vector
		self.vector2_slope = 0.0 # right side vector
		self.slope_threshold = 4 # to find the turnings, need to find the correct value
		self.obstacle_distance = 3.0  # it should be the max value for obstacle distance
		self.obstacle_distance_left = 3.0  # it should be the max value for obstacle distance
		self.obstacle_distance_right = 3.0  # it should be the max value for obstacle distance
		self.obstacle_distance_narrow_view = 3.0  # it should be the max value for obstacle distance


	### edited by saik
	def calculate_slope(self, point1, point2):
		if math.isclose(point1.x, point2.x, abs_tol=1e-9):
			return float('inf')  # Vertical line
		return (point2.y - point1.y) / (point2.x - point1.x)

	""" Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: the speed of the car in float. Range = [-1.0, +1.0];
				Direction: forward for positive, reverse for negative.
			turn: steer value of the car in float. Range = [-1.0, +1.0];
				Direction: left turn for positive, right turn for negative.

		Returns:
			None
	"""
	def rover_move_manual_mode(self, speed, turn):
		msg = Joy()

		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

		msg.axes = [0.0, speed, 0.0, turn]

		self.publisher_joy.publish(msg)

	""" Analyzes edge vectors received from /edge_vectors to achieve line follower application.
		It checks for existence of ramps & obstacles on the track through instance members.
			These instance members are updated by the lidar_callback using LIDAR data.
		The speed and turn are calculated to move the rover using rover_move_manual_mode.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

		Returns:
			None
	"""
	def edge_vectors_callback(self, message):
#		speed = SPEED_MAX
#		turn = TURN_MIN
		speed = self.current_speed
		turn = self.current_turn

		vectors = message
		half_width = vectors.image_width / 2

		# NOTE: participants may improve algorithm for line follower.
		if (vectors.vector_count == 0):  # nothing => no turn, speed have to decrease
			# speed = 0.4
			turn = 0.0 # for sake of debug
			# speed = max(SPEED_MIN, speed-0.1) # need to find the correct decrement value
			# turn = 0
			# pass

		if (vectors.vector_count == 1):  # curve => take turn when the vector touches the appropriate corner
#			# Calculate the magnitude of the x-component of the vector.
			deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
			turn = deviation / vectors.image_width
			
			### added by saik
			# find wether to turn or not right turn
			speed = 0.5 # need to find the correct speed value
			# turn = 0.0 # for sake of debug
			if self.right_turn_indication:
				if vectors.vector_1[1].x == vectors.image_width: #turn only when the vector head touches right border
					turn -= 1 # need to find the correct value
				else:
					turn = 0
			elif self.left_turn_indication:
				if vectors.vector_1[0].x == 0: # turn only when the vector tail touches left border
					turn += 1 # need to find the correct turn value
				else:
					turn = 0

		if (vectors.vector_count == 2):  # straight => detect the turns and move accordingly
			# Calculate the middle point of the x-components of the vectors. => this make sure to move the buggy in middle of the two vectors
			middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
			middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
			middle_x = (middle_x_left + middle_x_right) / 2
			deviation = half_width - middle_x
			turn = deviation / half_width
			
			### added by saik
			## below code is to detecting turnings in the road
			## changes the turning value and speed(????need to find it?????) if needed	
			#find the slope
			self.vector1_slope = self.calculate_slope(vectors.vector_1[0], vectors.vector_1[1])
			self.vector2_slope = self.calculate_slope(vectors.vector_2[0], vectors.vector_2[1])
   
			#finding the turns(left or right) when the slope changes out of the range(mine is left_slope = -4, right_slope = 4)
			self.left_turn_indication = 1 if self.vector1_slope < -self.slope_threshold else 0
			self.right_turn_indication = 1 if self.vector2_slope > self.slope_threshold else 0
   
			#finding the slopes if the slope is in the limits
			if not self.left_turn_indication and not self.right_turn_indication:
				#for now we are using the NXP's method need to change turn value
				# speed += 0.01 + speed*0.01 # need to change this value(speed increment)
				speed = min(SPEED_MAX, speed+0.01)
				# speed = SPEED_MAX # until obstacle distance test
				print("no turn")
			# elif self.left_turn_indication:
			# 	turn = 0
			# 	speed -= 0.01 # need to change this value
			# 	print("left turn ahead not turning speed decreasing")
			# elif self.right_turn_indication:
			# 	turn = 0
			# 	speed -= 0.01 # need to change this value
			# 	print("right turn ahead not turning speed decreasing")
			# else: 
			# 	print("this fucks")
			# if self.obstacle_detected is True:
			# 	self.ramp_detected = True
   
# 		if (self.traffic_status.stop_sign is True):
# 			speed = SPEED_MIN
# 			# print("stop sign detected")

		# decrease the speed at obstacle it will work for ramp also need to narrow down the lidar front view for ramp detection
		speed =  max(0.4, self.obstacle_distance_narrow_view - 1) if(0.2 < self.obstacle_distance_narrow_view < 1.5) else speed
# 		if self.ramp_detected is True:
# 			# TODO: participants need to decide action on detection of ramp/bridge.
# #			print("ramp/bridge detected")
# 			### added by saik
# 			speed = max(0.0, self.obstacle_distance - 0.25) # need to change this value
# 			print(f"ramp/bridge detected.  speed : {speed}")
			
   
# 		if self.obstacle_detected is True:
# 			# TODO: participants need to decide action on detection of obstacle.
# 			print(f"obstacle detected at {self.obstacle_distance}")
# 			### added by saik
# 			## self.obstacle_distance -> it is the distance from the closes object in the front region
# 			#speed = max(0.0, self.obstacle_distance-0.25) # need to change this value
# 			## we should get find a gap to turn the buggy and go 

		###for debuggin purpose
		print("------") 
		print("test 7")
		print(f"{vectors.vector_count} vector1 slope : {self.vector1_slope} vector2 slope : {self.vector2_slope}")
		print(f"left turn indication :{self.left_turn_indication} right turn indication {self.right_turn_indication}")
		print(f"speed :{speed} turn: {turn} obstacle detected:{self.obstacle_detected}")
		print(f"narrow: {self.obstacle_distance_narrow_view} center: {self.obstacle_distance} left: {self.obstacle_distance_left} right: {self.obstacle_distance_right}")
		print("------") 
  
		
		self.current_speed = speed
		self.current_turn = turn

		self.rover_move_manual_mode(speed, turn)

	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		self.traffic_status = message

	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""
	def lidar_callback(self, message):
		# TODO: participants need to implement logic for detection of ramps and obstacles.

		shield_vertical = 4
		shield_horizontal = 1
		theta = math.atan(shield_vertical / shield_horizontal)

		# Get the middle half of the ranges array returned by the LIDAR.
		length = float(len(message.ranges))
		ranges = message.ranges[int(length / 4): int(3 * length / 4)]

		# Separate the ranges into the part in the front and the part on the sides.
		length = float(len(ranges))
		front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
		side_ranges_right = ranges[0: int(length * theta / PI)]
		side_ranges_left = ranges[int(length * (PI - theta) / PI):]

		# this part is for ramp detection
		shield_vertical = 3
		shield_horizontal = 0.4
		theta = math.atan(shield_vertical / shield_horizontal)
		ramp_view_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]

#		# process front ranges.
#		angle = theta - PI / 2
#		for i in range(len(front_ranges)):
#			if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
#				self.obstacle_detected = True
#				self.obstacle_distance = front_ranges[i] ## added by sai
#				return
#
#			angle += message.angle_increment
#
#		# process side ranges.
#		side_ranges_left.reverse()
#		for side_ranges in [side_ranges_left, side_ranges_right]:
#			angle = 0.0
#			for i in range(len(side_ranges)):
#				if (side_ranges[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
#					self.obstacle_detected = True
#					return
#
#				angle += message.angle_increment
  
  
		### edited by sai
		# meed to find the minimum value and compare it with the threshold to find wether there is an obstacle or not
		for i in range(len(front_ranges)):
			self.obstacle_distance = front_ranges[i] ## added by sai
			if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
				self.obstacle_detected = True
				return

		for i in range(len(side_ranges_left)):
			self.obstacle_distance_left = side_ranges_left[i] ## added by sai
			if (side_ranges_left[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
				self.obstacle_detected = True
				return

		for i in range(len(side_ranges_right)):
			self.obstacle_distance_right = side_ranges_right[i] ## added by sai
			if (side_ranges_right[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
				self.obstacle_detected = True
				return

		# maybe helpfull in finding the ramp
		for i in range(len(ramp_view_ranges)):
			self.obstacle_distance_narrow_view = ramp_view_ranges[i] ## added by sai
			if (ramp_view_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
				self.obstacle_detected = True
				return

		self.obstacle_detected = False


def main(args=None):
	rclpy.init(args=args)

	line_follower = LineFollower()

	rclpy.spin(line_follower)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	line_follower.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
