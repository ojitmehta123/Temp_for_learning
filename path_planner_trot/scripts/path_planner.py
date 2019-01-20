#! /usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion,quaternion_from_euler
rospy.init_node("path_planner_trot")

yaw=0 
pos_x=0
pos_y=0
def callback_status_inform(msg):
		global roll,pitch,yaw,pos_x,pos_y
		#	rospy.loginfo(msg.pose.pose.orientation)
		pos_x=msg.pose.pose.position.x
		pos_y=msg.pose.pose.position.y
		(roll,pitch,yaw)=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
		rospy.loginfo("roll->%s \t pitch->%s \t yaw->%s",roll,pitch,yaw)
		#	rospy.loginfo(msg.twist)

class Planner():
	def __init__(self):
		
		self.path=[]
		self.completed=False
		self.present_state=rospy.Subscriber("odom",Odometry,callback_status_inform)
		self.move_there=rospy.Publisher("cmd_vel",Twist,queue_size=1)

	
		
	def get_path(self,input_path):
		"""
		input_path=[(x1,y1),(x2,y2)..]
		"""
		self.path=input_path
		print("path set to -->")
		print(self.path)
		
	def go_to_path(self,goal,start):
		"""
		Given goal and start travel there
		goal=(x2,y2)
		start=(x1,y1)		
		"""	
		twist_instance=Twist()
		delta_y=goal[1]-start[1]
		delta_x=goal[0]-start[0]
		theta=math.atan(delta_y/delta_x)
		mod_r=math.sqrt(((delta_x)**2+(delta_y)**2))
		
		while not rospy.is_shutdown():
			if((theta-yaw)<0.01 and (theta-yaw)>-0.01):
				if (((goal[0]-pos_x)**2+(goal[1]-pos_y)**2)<0.001):
					print("reached->"+str(pos_x)+","+str(pos_y))
					twist_instance_0=Twist()
					self.move_there.publish(twist_instance_0)		
					return (pos_x,pos_y)
#				twist_instance=Twist()
				twist_instance.linear.x=0.2
				twist_instance.angular.z=0
#				self.move_there.publish(twist_instance)		
			else:
				twist_instance.angular.z=(theta-yaw)/abs(theta-yaw)/5		
			self.move_there.publish(twist_instance)
		
	

		
if __name__=="__main__":
	inst=Planner()
	inst.get_path([(0,0),(0.5,0.5),(1,0)])
	inst.go_to_path((0.5,0.5),(0,0))
	print("dieing!")
	
	
		
		
		
