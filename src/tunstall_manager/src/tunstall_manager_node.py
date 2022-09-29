#!/usr/bin/env python
# import the necessary packages
import numpy as np
import rospy
import sys
import json
from enum import Enum
from std_msgs.msg import String

# TODO: 
# 
#class TSensorType (Enum):
 #   DOOR = auto()
 #   PIR = auto()
  #  CHAIR = auto()
   # UNKNOWN = auto()

class TSensor:
	sensor_id : 0
	sensor_position = (0.0,0.0)
	sensor_room = "Unknown"
	sensor_type = "Unknown"
	sensor_status = False

class tunstall_manager_node:
	def __init__(self) -> None:
		self.pub_alarm = rospy.Publisher('/tunstall/result', String, queue_size=10)
		self.sub = rospy.Subscriber('/tunstall/trigger', String, self.tunstall_trigger_callback)
		self.rate = rospy.Rate(1)
		self.sensor_database = None
		self.sensor_file = rospy.get_param('~sensorFile')

		self.load_sensors_from_file()

        # load sensor database from json file
        # sensors = {}
        # id ==> {place (dupla), type (enum), status (bool)}
		# example:
        # 	{ 
		# 		id : 1, 
		# 		position : (7.1, 3.4),
		# 		room : "2.3.6i", 
		# 		type : TSensorType.CHAIR,
		# 		status : False
		# 	}
		while not rospy.is_shutdown():
			self.rate.sleep()
	
	def tunstall_trigger_callback(self,data):
        # find the sensor in the database,
        # change its status and
        # according to its type, perfom an action (basically GO and RECOGNIZE and TALK)
		return
    
	def load_sensors_from_file(self):
		self.sensor_database = json.load(self.sensor_file)
		for i in self.sensor_database['emp_details']:
			print(i)



def main(args):
  rospy.init_node('tunstall_manager_node', anonymous=True)
  node = tunstall_manager_node()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)