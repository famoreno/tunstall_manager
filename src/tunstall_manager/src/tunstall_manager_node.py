#!/usr/bin/env python
# import the necessary packages
#from asyncio.windows_events import NULL
import numpy as np
from simplejson import JSONEncoder
import rospy
import sys
import json
from enum import Enum, auto
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
import datetime

from tunstall_manager.srv import command_tunstall_manager, command_delete_file

import time
def to_seconds(date):
    return time.mktime(date.timetuple())

# TODO: 
# 

# preguntar por los argumentos de estas clases
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
	sensor_status = (False,0)




class tunstall_manager_node:
	def __init__(self) -> None:
		self.pub_alarm = rospy.Publisher('/tunstall/result', String, queue_size=10)
		self.pub_task = rospy.Publisher('/mqtt2ros', KeyValue, queue_size=10)
		self.sub_trigger = rospy.Subscriber('/tunstall/trigger', String, self.tunstall_trigger_callback)
		self.sub_recognizer = rospy.Subscriber('/face_recognizer/name',String, self.face_recognized_callback)
		self.rate = rospy.Rate(1)
		self.sensor_database = None
		self.sensor_file = rospy.get_param('~sensorFile')
		self.verbose = rospy.get_param('~verbose')
		self.active = True
		self.escenario = "None"

		#Service to start/stop surveillance mode
		
		self.srv = rospy.Service('~command_tunstall_manager', command_tunstall_manager, self.handle_command_tunstall_manager)
		
		self.srv = rospy.Service('~command_delete_file',command_delete_file, self.handle_command_delete_file)

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
	
	def face_recognized_callback(self,msg):

		if self.escenario == "CAFE":
			## callback from the topic "nombre" 
			if msg == "Unknown":
				text_to_say = "Ese no es su asiento, levántese, por favor"

			elif msg == "Roberto":
				text_to_say = "tu café es el de la izquierda, sin azucar"

			elif msg == "Paco":
				text_to_say = "Tu café es el de la derecha, con leche sin lactosa"

			else: 
				text_to_say = "Lo siento, no traigo café para usted"

			# json to say something

		elif self.escenario == "CANSANCIO":
			text_to_say = "Deberías tomarte un descanso, " + msg
			
			# json to say something


	def tunstall_trigger_callback(self,msg):
		 if self.active:
			#
				nombre = self.nombre


				split_data = msg.data.split("_")
				code_id= split_data[0]
				code = code_id[0:2]
				print(code)

				id = code_id[2:4]
				print(id)

				date_time_str = split_data[1]
				# print(type(date_time_str))
				date_time_str = date_time_str[1:27]
				# print(date_time_str)

				# BH14_{2022-10-17 12:13:19.1180003}
				# {2022-10-17 12:13:19.1180003}
				# {%YYYY-%mm-%dd %HH:%MM:%SS.%fffffff}

				#TODO: por que carajo no va esto
				#if self.verbose:
				#	rospy.loginfo("[tunstall_manager_node] Code Received: ",str(code))
				#	rospy.loginfo("[tunstall_manager_node] Date Received: ",date_time_str)
					
				date_time_obj = datetime.datetime.strptime(date_time_str, "%Y-%m-%d %H:%M:%S.%f")
				timestamp = to_seconds(date_time_obj)
				#timestamp = datetime.timestamp(date_time_obj)

				if code == "AR" : 
					#sensor['type'] = "DOOR"
					if self.check_type(id,"DOOR"):
						aux = False
						self.add_to_history(aux, timestamp, id)
					else:
						print("Warning: This kind of sensor does not use this code")

				elif code == "AQ" : 
					#sensor['type'] = "DOOR"
					if self.check_type(id,"DOOR"):
						aux = True
						self.add_to_history(aux, timestamp, id)
					else:
						print("Warning: This kind of sensor does not use this code")
				

				elif code == "BA" : 
					#sensor['type'] = "CHAIR"
					if self.check_type(id,"CHAIR"):
						aux = True
						self.add_to_history(aux, timestamp, id)
					else:
						print("Warning: This kind of sensor does not use this code")

				elif code == "AZ" : 
					#sensor['type'] = "CHAIR"
					if self.check_type(id,"CHAIR"):
						aux = False
						self.add_to_history(aux, timestamp, id)
					else:
						print("Warning: This kind of sensor does not use this code")
					

				elif code == "BH" : 
					#sensor['type'] = "PIR"
					if self.check_type(id,"PIR"):
						aux = True
						self.add_to_history(aux, timestamp, id)
					else:
						print("Warning: This kind of sensor does not use this code")



				#if the chair sensor is activated go there and say something
				if code == "BA":
					if id == "03":
						desired_id = "03"
						target = "paco"
					elif id == "02":
						desired_id = "02"
						target = "roberto"

					for sensor in self.sensor_db_dict['sensor_db']:
						if sensor["id"] == desired_id and (((sensor["status"][-1][2]-sensor["status"][-2][2])/3600)>1):

							# check if the door is open
							if id == "03":
								desired_id = "00"
								target = "paco"

							elif id == "02":
								desired_id = "01"
								target = "roberto"

							for sensor in self.sensor_db_dict['sensor_db']:
								if sensor["id"] == desired_id and sensor ["status"][-1][1] == True:
									# if the door is open the scenario is "CANSANCIO"

									## go to to the position of the chair sensor with some correction
									# json message with the information to move

									## activate face_detection_node 
									# json message to activate face_detection_node

									## activate speak node
									# json message to activate sepak_node
									self.escenario = "CANSANCIO"


								else :

									self.escenario = "PUERTA BLOQUEA EL PASO"

									## activate speak node
									# json message to activate sepak_node

									

					# if the robot is neither in "CANSANCIO" scenario nor in "PUERTA BLOQUEA EL PASO" scenario
					# check if it is in "C"
					if self.escenario != "CANSANCIO" and self.escenario != "PUERTA BLOQUEA EL PASO":
						# id:03 - id:00
					#	if id == "03":
					#		desired_id = "00"
					#		target = "paco"
					#	elif id == "02":
					#		desired_id = "01"
					#		target = "roberto"
					#	for sensor in self.sensor_db_dict['sensor_db']:
					#		if sensor["id"] == desired_id and sensor ["status"][-1][1] == True:


								## go to to the position of the chair sensor with some correction
								# json message with the information to move

								## activate the face_detection_node node
								# json message to activate face_detection_node
								self.escenario = "CAFE"

				

					
					

					
				
					
	def check_type(self, id, in_type):
		for sensor in self.sensor_db_dict['sensor_db']:
			if int(id) == sensor['id']:
				print("sensor['type']: ", sensor['type'] )
				return sensor['type'] == in_type
		#return False
					
	def add_to_history(self, aux, timestamp, id):

		encontrado = False
		for sensor in self.sensor_db_dict['sensor_db']:
			# print("id: ", type(int(id)))
			# print("sensor id: ", type(sensor['id']))

			if int(id) == sensor['id']:
				sensor['status'] = [aux,timestamp]
				encontrado = True
				if self.verbose:
					# for debugging
					print(sensor) 
				break
						
			if not encontrado:
				rospy.loginfo("[tunstall_manager_node] ERROR: id not found")
					

        # find the sensor in the database,
        # change its status and
        # according to its type, perfom an action (basically GO and RECOGNIZE and TALK)
		
		#return 
    
	def load_sensors_from_file(self):
		json_file = open(self.sensor_file,"r")
		self.sensor_db_dict = json.load(json_file)
		#self.sensor_db_list = json.load(json_file)[sensor_db]
		json_file.close()

		for sensor in self.sensor_db_dict['sensor_db']:
			# for debugging
			print(sensor) 
			self.string_to_other(sensor)
			print(sensor) # for debug

			# convert to TSensor and add to the database
			# self.sensor_database[sensor["id"]]
			
	# auxiliary methods
	def string_to_other(self,sensor):
		if sensor['type'] == 'CHAIR' : 
			sensor['type'] = TSensorType.CHAIR
		elif sensor['type'] == 'DOOR':
			sensor['type'] = TSensorType.DOOR
		elif sensor['type'] == 'PIR':
			sensor['type'] = TSensorType.PIR
		else:
			sensor['type'] = TSensorType.UNKNOWN

	def other_to_string(self, sensor):

		# auxiliary method to convert sensor variable to a string type
		if sensor['type'] == TSensorType.CHAIR : 
			sensor['type'] = 'CHAIR'
		elif sensor['type'] == TSensorType.DOOR:
			sensor['type'] = 'DOOR'
		elif sensor['type'] == TSensorType.PIR:
			sensor['type'] = 'PIR'
		elif sensor['type'] == TSensorType.UNKNOWN:
			sensor['type'] = 'UNKNOWN'

	def save_db(self):
		json_file = open(self.sensor_file,"w")
		for sensor in self.sensor_db_dict['sensor_db']:
			# for debugging
			print(sensor) 
			self.other_to_string(sensor)
			print(sensor) # for debug
		json.dump(self.sensor_db_dict, json_file)
		json_file.close()

	
	def handle_command_tunstall_manager(self,req):
		#Check whether we start ase = None
		self.sensor_file = rospy.get_param('~sensorFile')
		self.verbose = rospy.get_param('~verbose')
		self.active = True

		#Service to start/stop surveillance mode
		
		self.srv = rospy.Service('~command_tunstall_manager', command_tunstall_manager, self.handle_command_tunstall_manager)
		
		self.srv = rospy.Service('~command_delete_file',command_delete_file, self.handle_command_delete_file)

		self.load_sensors_from_file()

        # load sensor the node or we stop it
		if req.task_command == "on":
			self.active = True
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: ON")

			return True


		#Check if we save the state of the sensor
		elif req.task_command == "save":
			
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: SAVE")

			# Saving the data on a Json file
			self.save_db()
			
			return True
			
			
		
		#check if we delete all history
		elif req.task_command == "restart":
			
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Recieved command: RESTART")

			# Restarting database
			#json_file = open(self.sensor_file,"w")
			#self.sensor_db_dict = json_file

			for sensor in self.sensor_db_dict['sensor_db']:
				#sensorValue is a custom variable that contains if the sensor is  on/off at any given moment
				sensor ["status"] = []
				if self.verbose:
					print(sensor)

			#if self.verbose:
			#	for sensor in self.sensor_db_dict['sensor_db']:
			#		# for debugging
			#		print(sensor) 
			#		self.set_sensor_type(sensor)
			#		print(sensor) # for debug

			return True	

			#json.dump(self.sensor_db_dict, json_file)
			#json_file.close()


		elif req.task_command == "off":
			self.active = False
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: OFF")
				
			return True

		else: 
			return False

	def handle_command_delete_file(self,req):

		

			#check if we want delete a single element
		if req.task_command == "delete":
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: DELETE")
			
			#if self.sensor_db_dict['sensor_db'][req.id] == None:

			#	rospy.loginfo("[tunstall_manager_node] ERROR: id not found")
			#else:

				#self.sensor_db_dict['sensor_db'][req.id]['status'] = []

			
				encontrado = False
				for sensor in self.sensor_db_dict['sensor_db']:

					if req.id == sensor['id']:
						sensor['status'] = []
						encontrado = True
						if self.verbose:
							# for debugging
							print(sensor) 
						break
					
				if not encontrado:
					rospy.loginfo("[tunstall_manager_node] ERROR: id not found")
					
			return True
		else:
			return False

		

def main(args):
  rospy.init_node('tunstall_manager_node', anonymous=True)
  node = tunstall_manager_node()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)