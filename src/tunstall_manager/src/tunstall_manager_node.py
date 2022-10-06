#!/usr/bin/env python
# import the necessary packages
import numpy as np
import rospy
import sys
import json
from enum import Enum, auto
from std_msgs.msg import String

from tunstall_manager.srv import command_tunstall_manager,command_save_file,command_delete_all_history,command_delete_single_history

# TODO: 
# 
class TSensorType (Enum):
	DOOR = auto()
	PIR = auto()
	CHAIR = auto()
	UNKNOWN = auto()

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

		#Service to start/stop surveillance mode
		
		self.srv = rospy.Service('~command_tunstall_manager', command_tunstall_manager, self.handle_command_tunstall_manager)
		
		#Service to register sensor changes

		self.srv = rospy.service('~command_save_file', command_save_file, self.handle_command_save_file)

		#Service to restart databse

		self.srv = rospy.service('~command_delete_all_history', command_delete_all_history, self.handle_command_delete_all_history)

		#Service to delete a single element of the history

		self.srv = rospy.service('~command_delete_single_history', command_delete_single_history, self. handle_command_delete_single_history)

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
		json_file = open(self.sensor_file,"r")
		self.sensor_db_dict = json.load(json_file)
		json_file.close()

		for sensor in self.sensor_db_dict['sensor_db']:
			# for debugging
			print(sensor) 
			self.set_sensor_type(sensor)
			print(sensor) # for debug

			# convert to TSensor and add to the database
			# self.sensor_database[sensor["id"]]
			
	# auxiliary methods
	def set_sensor_type(self,sensor):
		if sensor['type'] == 'CHAIR' : 
			sensor['type'] = TSensorType.CHAIR
		elif sensor['type'] == 'DOOR':
			sensor['type'] = TSensorType.DOOR
		elif sensor['type'] == 'PIR':
			sensor['type'] = TSensorType.PIR
		else:
			sensor['type'] = TSensorType.UNKNOWN

	def handle_command_tunstall_manager(self,req):

		#Check whether we start the node or we stop it
		if req.task_command == "on":
			self.active = True
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: ON")
				
				return True

		elif req.task_command == "off":
			self.active = False
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: OFF")
				
			return True
		else:
			return False

	def handle_command_save_file(self,sensor,req):
		#Check if we save the state of the sensor
		if req.task_command == "save":
			self.active = True
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Recieved command: SAVE")

				return True
			
			# auxiliary method to convert sensor variable to a string type
			if sensor['type'] == TSensorType.CHAIR : 
				sensor['type'] = 'CHAIR'
			elif sensor['type'] == TSensorType.DOOR:
				sensor['type'] = 'DOOR'
			elif sensor['type'] == TSensorType.PIR:
				sensor['type'] = 'PIR'
			elif sensor['type'] == TSensorType.UNKNOWN:
				sensor['type'] = 'UNKOWN'

			# Saving the data on a Json file
			json_file = open(self.sensor_file,"w")
			self.sensor_db_dict = json_file

			for sensor in self.sensor_db_dict['sensor_db']:
				#sensorValue is a custom variable that contains if the sensor is  on/off at any given moment
				sensor ["status"] = sensorValue

			json.dump(self.sensor_db_dict, json_file)
			json_file.close()

		else: 
			return False

	def handle_command_delete_all_history(req,self,sensor):
		#check if we delete all history
		if req.task_command == "restart":
			self.active = True
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Recieved command: RESTART")

				return True

		# auxiliary method to convert sensor variable to a string type
			if sensor['type'] == TSensorType.CHAIR : 
				sensor['type'] = 'CHAIR'
			elif sensor['type'] == TSensorType.DOOR:
				sensor['type'] = 'DOOR'
			elif sensor['type'] == TSensorType.PIR:
				sensor['type'] = 'PIR'
			elif sensor['type'] == TSensorType.UNKNOWN:
				sensor['type'] = 'UNKOWN'

		# Restarting database
			json_file = open(self.sensor_file,"w")
			self.sensor_db_dict = json_file

			for sensor in self.sensor_db_dict['sensor_db']:
				#sensorValue is a custom variable that contains if the sensor is  on/off at any given moment
				sensor ["status"] = False

			json.dump(self.sensor_db_dict, json_file)
			json_file.close()

	def handle_command_delete_single_history(req,sensor,self):
		#check if we want delete a single element
		if req.task_command == "delete":
			self.active = True
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Recieved command: DELETE")

				return True
			
			# auxiliary method to convert sensor variable to a string type
			if sensor['type'] == TSensorType.CHAIR : 
				sensor['type'] = 'CHAIR'
			elif sensor['type'] == TSensorType.DOOR:
				sensor['type'] = 'DOOR'
			elif sensor['type'] == TSensorType.PIR:
				sensor['type'] = 'PIR'
			elif sensor['type'] == TSensorType.UNKNOWN:
				sensor['type'] = 'UNKOWN'

			# deleting file
			json_file = open(self.sensor_file,"w")
			self.sensor_db_dict = json_file

			

			




			
			


def main(args):
  rospy.init_node('tunstall_manager_node', anonymous=True)
  node = tunstall_manager_node()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)