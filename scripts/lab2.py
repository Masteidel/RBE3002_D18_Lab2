#!/usr/bin/env python
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class Robot:
	
	def __init__(self):

		"""
			This constructor sets up class variables and pubs/subs
		"""
	
		self._current = Pose() # initlize correctly 
		self._odom_list = tf.TransformListener()
		rospy.Timer(rospy.Duration(.1), self.timerCallback)
		self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1) # handle nav goal events


	def navToPose(self,goal):
		"""
			This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and 
			then spin to match the goal orientation.
		"""
		origin = copy.deepcopy(self._current)
		self._odom_list.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
		transGoal = self._odom_list.transformPose('/base_link', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system
		goalPoseX = transGoal.pose.position.x	#x position of the goal
		goalPoseY = transGoal.pose.position.y	#y position of the goal
		odomW = transGoal.pose.orientation
		q1 = [odomW.x, odomW.y, odomW.z, odomW.w]
		roll1, pitch1, yaw1 = euler_from_quaternion(q1)
		goalPoseAng = yaw1					#orientation of goal
		initialX = origin.position.x				#Starting x position of turtlebot
		initialY = origin.position.y				#Starting y position of turtlebot
		#Rotate towards goal
		print goalPoseX
		print initialX
		if((goalPoseX - initialX) == 0):
			if((goalPoseY - initialY) > 0):
				print "spin!"
				self.rotate(math.pi)
			elif((goalPoseY - initialY) < 0):
				print "spin!"
				self.rotate(-math.pi)
		else:
			print "spin!"
			self.rotate(math.atan2((goalPoseY - initialY), (goalPoseX - initialX)))
		#Drive towards goal
		print "move!"
		self.driveStraight(0.2, math.sqrt(math.pow((goalPoseX - initialX), 2) + math.pow((goalPoseY - initialY), 2)))
		q2 = [self._current.orientation.x,
			 self._current.orientation.y,
			 self._current.orientation.z,
			 self._current.orientation.w]
		roll2, pitch2, yaw2 = euler_from_quaternion(q2)
		initialAng = yaw2	#Heading of turtlebot after reaching desired location
		#Rotate to pose
		if((goalPoseAng - initialAng) != 0):
			if((goalPoseAng - initialAng) > math.pi):
				print "spin!"
				self.rotate((goalPoseAng - initialAng) - 2*math.pi)
			elif((goalPoseAng - initialAng) < -math.pi):
				print "spin!"
				self.rotate((goalPoseAng - initialAng) + 2*math.pi)
			else:
				print "spin!"
				self.rotate(goalPoseAng - initialAng)
		print "done"

	def executeTrajectory(self):
		self.driveStraight(0.2, 0.6)
		self.rotate(-math.pi/2)
		self.driveStraight(0.2, 0.45)
		self.rotate(3*math.pi/4)

	def driveStraight(self, speed, distance):
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		origin = copy.deepcopy(self._current) #hint:  use this

		initialX = self._current.position.x
		initialY = self._current.position.y

		drive_msg = Twist()
		stop_msg = Twist()

		drive_msg.linear.x = speed
		stop_msg.linear.x = 0

		atTarget = False
		while(not atTarget and not rospy.is_shutdown()):
			currentX = self._current.position.x
			currentY = self._current.position.y
			print currentX
			print currentY
			currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
			
			if(currentDistance >= distance):
				atTarget = True
				pub.publish(stop_msg)
			else:
				pub.publish(drive_msg)
				rospy.sleep(0.15)
		
		
	def spinWheels(self, v_left, v_right, time):
		wheelbase = 0.16 # based on wheel track from http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#specifications

		lin_vel = (v_right + v_left)/2
		ang_vel = (v_right - v_left)/wheelbase

		twist_msg = Twist()
		stop_msg = Twist()

		twist_msg.linear.x = lin_vel
		twist_msg.angular.z = ang_vel
		stop_msg.linear.x = 0
		stop_msg.angular.z = 0

		startTime = rospy.Time.now().secs

		while(rospy.Time.now().secs - startTime <= time and not rospy.is_shutdown()):
			self._vel_pub.publish(twist_msg)
		self._vel_pub.publish(stop_msg)
		

	def rotate(self,angle):
		origin = copy.deepcopy(self._current)

		q = [self._current.orientation.x,
			 self._current.orientation.y,
			 self._current.orientation.z,
			 self._current.orientation.w] # quaternion nonsense

		(roll, pitch, yaw) = euler_from_quaternion(q)
		
		if(angle > math.pi or angle < -math.pi):
			print "angle is too large or too small"
		else:
			vel = Twist()
			done = False
			print angle
			initialHeading = yaw
			if(angle > 0):
				vel.angular.z = 1
			else:
				vel.angular.z = -1

			while(not done and not rospy.is_shutdown()):
				q = [origin.orientation.x,
					 origin.orientation.y,
					 origin.orientation.z,
					 origin.orientation.w]
				(roll, pitch, yaw) = euler_from_quaternion(q)
				currentHeading = yaw
				print currentHeading
				diff = currentHeading - initialHeading

				if(diff > math.pi):
					error = angle - (diff - 2*math.pi)
				elif(diff < math.pi):
					error = angle - (diff + 2*math.pi)
				else:
					error = angle - diff

				if(abs(error) >= math.radians(2.0)):
					self._vel_pub.publish(vel)
				else:
					done = True
					vel.angular.z = 0
					self._vel_pub.publish(vel)


	def timerCallback(self,evprent):
		"""
			This is a callback that runs every 0.1s.
			Updates this instance of Robot's internal position variable (self._current)
		"""
		# wait for and get the transform between two frames
		self._odom_list.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
		(position, orientation) = self._odom_list.lookupTransform('/odom','/base_link', rospy.Time(0)) 
		# save the current position and orientation
		self._current.position.x = position[0]
		self._current.position.y = position[1]
		self._current.orientation.x = orientation[0]
		self._current.orientation.y = orientation[1]
		self._current.orientation.z = orientation[2]
		self._current.orientation.w = orientation[3]
		
		# create a quaternion
		q = [self._current.orientation.x,
			 self._current.orientation.y,
			 self._current.orientation.z,
			 self._current.orientation.w] 

	# convert the quaternion to roll pitch yaw
		(roll, pitch, yaw) = euler_from_quaternion(q)    
		

	# helper functions
	def planTraj(self, b, t):
		"""
			Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
		"""
		

if __name__ == '__main__':
	
	rospy.init_node('drive_base')
	turtle = Robot()

	turtle.rotate(-math.pi/2)
	
	while  not rospy.is_shutdown():
		pass    
