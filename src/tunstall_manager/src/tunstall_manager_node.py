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

class TScenario (Enum):
	NONE = auto()
	CAFE = auto()
	CANSANCIO = auto()
	SEGURIDAD = auto()
	PUERTA_CERRADA = auto()

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


''' MAIN CLASS'''
class tunstall_manager_node:
	def __init__(self) -> None:
		self.pub_alarm = rospy.Publisher('/tunstall/result', String, queue_size=10)
		self.pub_task = rospy.Publisher('/mqtt2ros', KeyValue, queue_size=10)
		self.sub_trigger = rospy.Subscriber('/tunstall/trigger', String, self.tunstall_trigger_callback)
		self.sub_recognizer = rospy.Subscriber('/face_recognizer/name',String, self.face_recognized_callback)
		self.rate = rospy.Rate(1)
		self.sensor_database = None
		self.db_counter = 0

		self.sensor_file = rospy.get_param('~sensorFile')
		self.move_command_file = rospy.get_param('~move_command_file')
		self.speak_command_file = rospy.get_param('~speak_command_file')
		self.face_detect_command_file = rospy.get_param('~face_detect_command_file')
		self.wait_command_file = rospy.get_param('~wait_command_file')

		self.verbose = rospy.get_param('~verbose')
		self.active = True
		self.scenario = TScenario.NONE
		self.time_threshold = 0.05 # hours

		self.check_door_counter = 0

		# activate timer for each 5 minutes
		rospy.Timer(rospy.Duration(5*60), self.cansancio_timer_callback)
		
		# services for handling commands
		self.srv_cmd = rospy.Service('~command_tunstall_manager', command_tunstall_manager, self.handle_command_tunstall_manager)
		self.srv_del = rospy.Service('~command_delete_file', command_delete_file, self.handle_command_delete_file)

		# load info from files
		self.load_sensors_from_file()
		self.load_speak_command_from_file()
		self.load_move_command_from_file()
		self.load_face_detect_command_from_file()
		self.load_wait_command_from_file()
		
        # load sensor database from json file
        # sensors = {}
        # id ==> {place (dupla), type (enum), status (bool)}
		# example:
        # 	{ 
		# 		id : 1, 
		# 		position : (7.1, 3.4),
		# 		room : "2.3.6i",
		# 		name : "Paco" 
		# 		type : TSensorType.CHAIR,
		# 		status : [[False, 0]]
		# 	}
		while not rospy.is_shutdown():
			self.rate.sleep()
	
	''' CALLBACK METHODS '''
	# callback for the node services:
	# 	on/off
	# 	save history
	# 	delete all sensors history
	def handle_command_tunstall_manager(self,req):

        # start node processing
		if req.task_command == "on":
			self.active = True
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: ON")

			return True

		# save the sensor information to file
		elif req.task_command == "save":
			
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: SAVE")

			# Saving the data on a Json file
			self.save_db()
			
			return True
			
		
		# delete all sensors history
		elif req.task_command == "restart":
			
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: RESTART")

			for sensor in self.sensor_db_dict['sensor_db']:
				sensor ["status"] = []
				if self.verbose:
					print(sensor)

			return True	


		# pause the node processing
		elif req.task_command == "off":
			self.active = False
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: OFF")
				
			return True

		else: 
			return False

	# callback for service 'delete' history for specific sensor
	def handle_command_delete_file(self,req):

		# check if we want delete a single element
		if req.task_command == "delete":
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Received command: DELETE")
			
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

	# callback to check CANSANCIO
	def cansancio_timer_callback(self, event):
		
		# Do this only if nothing else is going on
		if self.scenario != TScenario.NONE:
			return
		
		rospy.loginfo("[tunstall_manager_node] Cansancio WATCHDOG triggered")
		
		# Check time for all CHAIR sensors and trigger alarm if needed
		for sensor in self.sensor_db_dict['sensor_db']:
			if sensor["type"] == TSensorType.CHAIR:
				first_room = sensor["room"]
				first_name = sensor["name"]
				is_active  = sensor["status"][-1][0]
				time_difference = (rospy.get_time()-sensor["status"][-1][1])/3600

				print(f'[tunstall_manager_node] {first_name} [{sensor["id"]}] has been {is_active} for {time_difference} hours')
				
				if is_active and (time_difference > self.time_threshold):
					# this has been sitting for more than one hour
					if self.verbose:
						rospy.loginfo("[tunstall_manager_node] Chair TIMER: more than " + str(self.time_threshold) + " hours has passed")

					# Find the corresponding door
					for sensor in self.sensor_db_dict['sensor_db']:
						if sensor["type"] == TSensorType.DOOR and sensor["room"] == first_room:
							my_door = sensor
							is_open = not my_door["status"][-1][0]
							found = True
							break

					if found:	
						self.scenario = TScenario.CANSANCIO

						if is_open:
							# puerta abierta							
							rospy.loginfo("[tunstall_manager_node] Door is open")
							self.send_move_command(first_name)
							self.send_face_detect_command("on")

						else:
							# puerta cerrada
							rospy.loginfo("[tunstall_manager_node] Door closed, I need help to open it")
							self.send_speak_command("I need someone to open " + first_name + "'s door")

							# save data for later processing
							self.linked_door_id = my_door["id"]
							self.target = first_name
							
							# update scenario
							# self.scenario = TScenario.PUERTA_CERRADA

							# Launch a timer to check the door every 2 seconds (up to 30s)
							self.check_door_timer = rospy.Timer(rospy.Duration(2), self.check_door_timer_callback)

			##					

	# callback for timer for checking if a door is open
	def check_door_timer_callback(self, event):
		rospy.loginfo("[tunstall_manager_node] Check door WATCHDOG triggered")

		# get last status of the door
		linked_door_sensor = self.find_sensor_by_id(self.linked_door_id)
		is_open = not linked_door_sensor["status"][-1][0]

		if is_open:
			# stop timer
			self.check_door_timer.shutdown()

			# Go to to the position of the chair sensor
			self.send_move_command(self.target)							
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Door open! Moving to " + self.target + "'s chair")

			# Activate face_detection_node 
			self.send_face_detect_command("on")
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Sendind detect face command")
			
			return

		# check up to 15 times
		self.check_door_counter += 1
		if self.check_door_counter > 15:
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] Check door timer TIMEOUT")

			self.check_door_counter = 0
			self.check_door_timer.shutdown()
		
			# cancel scenario
			self.scenario = TScenario.NONE

	# process face recognition result
	def face_recognized_callback(self,msg):

		if self.verbose:
			rospy.loginfo("[tunstall_manager_node] Recognized name: " + msg.data)

		text_to_say = ""
		if self.scenario == TScenario.CAFE:
			# callback from the topic "face_recognizer/nombre" 
			if str(msg.data) == "desconocido":
				text_to_say = "This is not your seat, please stand up"

			elif str(msg.data) == "Roberto":
				text_to_say = "Your coffee is in the red mug, without sugar, as you like"

			elif str(msg.data) == "Paco":
				text_to_say = "Your coffee is in the white mug, with lactose free milk "

			else: 
				text_to_say = "I am sorry, I do not have a coffee for you"

		elif self.scenario == TScenario.CANSANCIO:
			if str(msg.data) == "desconocido":
				text_to_say = "You should take a rest"
			else:
				text_to_say = "You should take a rest, " + str(msg.data)

		elif self.scenario == TScenario.SEGURIDAD:

			too_late = "03:00:00"
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] too late is after " + too_late)
			too_early = "07:00:00"
			if self.verbose:
				rospy.loginfo("[tunstall_manager_node] too early is before " + too_early)

			if str(msg.data) == "Roberto":
				current_time =self.get_time_zone()
				if current_time> too_late or current_time < too_early:
					rospy.loginfo("[tunstall_manager_node] Current time zone is dangerous")

					text_to_say = str(msg.data)+" You do not have permission to be here " + current_time

				else:
					for sensor in self.sensor_db_dict['sensor_db']:
						if sensor["type"] == TSensorType.CHAIR:	
							if sensor["status"][-1][0]:
								if sensor["name"] == "Roberto":			
									text_to_say = str(msg.data)+" There is someone in your chair"
								elif sensor["name"] == "Paco":			
									text_to_say = str(msg.data)+" If you are loocking for Paco, he is in his chair"


			elif str(msg.data) == "Paco":
				current_time =self.get_time_zone()
				if current_time> too_late or current_time < too_early:
					rospy.loginfo("[tunstall_manager_node] Current time zone is dangerous")

					text_to_say = str(msg.data)+"Do you have permission to be here at " + current_time + "?"

				else:
					for sensor in self.sensor_db_dict['sensor_db']:
						if sensor["type"] == TSensorType.CHAIR:	
							if sensor["status"][-1][0]:
								if sensor["name"] == "Paco":			
									text_to_say = str(msg.data)+" There is someone in your chair"
								elif sensor["name"] == "Roberto":			
									text_to_say = str(msg.data)+" If you are loocking for Roberto, he is in his chair"
				
			elif str(msg.data) == "Desconocido": 
				current_time =self.get_time_zone()
				if current_time> too_late or current_time < too_early:
					rospy.loginfo("[tunstall_manager_node] Current time zone is dangerous")

					text_to_say = str(msg.data)+" Leave now or i will call the security guard"

				else:	

					for sensor in self.sensor_db_dict['sensor_db']:
						if sensor["type"] == TSensorType.CHAIR:	
							if sensor["status"][-1][0]:
								if sensor["name"] == "Paco":
									text_to_say = "If you are loocking for Paco, he is in his chair"	
								elif sensor["name"] == "Roberto":	
									text_to_say = str(msg.data)+"If you are loocking for Roberto, he is in his chair"


		# Now speak! and go back to the docking station
		self.send_speak_command(text_to_say)
		rospy.loginfo("[tunstall_manager_node]" + text_to_say)
		self.send_move_command("docking_station")

		# Reset state
		self.scenario = TScenario.NONE

	# a sensor has been triggered
	def tunstall_trigger_callback(self,msg):

		if not self.active:
			return

		split_data = msg.data.split("_")
		code_id = split_data[0]
		code = code_id[0:2]
		id = code_id[2:4]

		if self.verbose:
			rospy.loginfo("[tunstall_manager_node] Received tunstall trigger: " + str(code) + ", " + str(id))

		date_time_str = split_data[1]
		date_time_str = date_time_str[1:27]
		
		date_time_obj = datetime.datetime.strptime(date_time_str, "%Y-%m-%d %H:%M:%S.%f")
		timestamp = to_seconds(date_time_obj)

		# SCENARIOS:
		# -- [1] CAFE: A chair is activated, the robot goes there with two cups, recognizes the person and offers their coffee.
		# -- [2] CANSANCIO: Every 5 minutes, the node checks if any chair has been activated for more than one hour. If so, goes to ask him to get some exercise. (No face recognition) ALthough it could use face recognition to say de person's
		# -- [3] SECURITY: All chairs are activated, the main door is open and the PIR is activated. The robot goes to the PIR, recognizes the face and ask for security if needed.
		# -- Secondary: in all scenarios, if the robot needs to traverse a closed door, it asks for help, waits some time and if nobody opens it then cancels the mission.
		
		# a door has been opened
		if code == "AR" : 
			if self.check_type(id,TSensorType.DOOR):
				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] Door: " + str(id) + " has been opened")
				
				aux = False
				self.add_to_history(aux, timestamp, id)
			else:
				rospy.logwarn("[tunstall_manager_node] Warning: This kind of sensor does not use this code")

		# a door has been closed
		elif code == "AQ" : 
			if self.check_type(id,TSensorType.DOOR):
				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] Door: " + str(id) + " has been closed")
				
				aux = True
				self.add_to_history(aux, timestamp, id)
			else:
				rospy.logwarn("[tunstall_manager_node] Warning: This kind of sensor does not use this code")
		
		# someone has sit
		elif code == "BA" : 
			if self.check_type(id,TSensorType.CHAIR):
				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] Chair: " + str(id) + " has been activated")

				aux = True
				self.add_to_history(aux, timestamp, id)
			else:
				rospy.logwarn("[tunstall_manager_node] Warning: This kind of sensor does not use this code")

		# someone has stood up
		elif code == "AZ" : 
			if self.check_type(id,TSensorType.CHAIR):
				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] Chair: " + str(id) + " has been deactivated")

				aux = False
				self.add_to_history(aux, timestamp, id)
			else:
				rospy.logwarn("[tunstall_manager_node] Warning: This kind of sensor does not use this code")

		# someone has passed the way: PIR activated
		elif code == "BH" : 
			if self.check_type(id,TSensorType.PIR):
				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] PIR: " + str(id) + " has been activated")

				aux = True
				self.add_to_history(aux, timestamp, id)
			else:
				rospy.logwarn("[tunstall_manager_node] Warning: This kind of sensor does not use this code")

		# STATE MACHINE
		if self.scenario == TScenario.NONE:
			if  code == "BA":
				self.scenario = TScenario.CAFE 

				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] Pasando a escenario CAFE ...")

				my_sensor = self.find_sensor_by_id(id)
				first_room = my_sensor["room"]
				first_name = my_sensor["name"]

				# Find the corresponding door
				for sensor in self.sensor_db_dict['sensor_db']:
					if sensor["type"] == TSensorType.DOOR and sensor["room"] == first_room:
						my_door = sensor
						is_open = not my_door["status"][-1][0]
						found = True
						break
		
				if found:	
					if is_open:
						# puerta abierta							
						rospy.loginfo("[tunstall_manager_node] Puerta abierta")
						self.send_move_command(first_name)
						self.send_face_detect_command("on")

					else:
						# puerta cerrada
						rospy.loginfo("[tunstall_manager_node] Puerta cerrada, necesito que me abran la puerta")
						self.send_speak_command("Necesito que me abran la puerta de " + first_name)

						# save data for later processing
						self.linked_door_id = my_door["id"]
						self.target = first_name
						
						# update scenario
						# self.scenario = TScenario.PUERTA_CERRADA

						# Launch a timer to check the door every 2 seconds (up to 30s)
						self.check_door_timer = rospy.Timer(rospy.Duration(2), self.check_door_timer_callback)
				

			if code == 	"BH":
				self.scenario = TScenario.SEGURIDAD
				#count = 0

				if self.check_type(id,TSensorType.PIR):
					if self.verbose:
						rospy.loginfo("[tunstall_manager_node] PIR: " + str(id) + " has been activated")

				self.send_move_command("PIR")
				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] Dirigiéndose al PIR... ")
				
				if self.verbose:
					rospy.loginfo("[tunstall_manager_node] Detectando cara...")
				self.send_face_detect_command("on")
				if self.verbose:
					rospy.loginfo("[tunstall_manager_mode] Elaborando una respuesta... ")
				
	''' AUXILIARY METHODS '''
	# builds a JSON message to be sent to the robot for talking
	def send_speak_command(self, text):
		# complete SAY command
		self.speak_command["data"]["text"]["talkcode"] = text
		self.speak_command["time"]["t"] = rospy.get_time()
		
		# create message and publish it
		msg = KeyValue()
		msg.key = "interventions/famd/INFO"
		msg.value = json.dumps(self.speak_command)
		self.pub_task.publish(msg)

	# builds a JSON message to be sent to the robot for going somewhere
	def send_move_command(self, place):
		# complete GO command
		self.move_command["data"]["label"] = place
		self.move_command["time"]["t"] = rospy.get_time()
		
		# create message and publish it
		msg = KeyValue()
		msg.key = "interventions/famd/INFO"
		msg.value = json.dumps(self.move_command)
		self.pub_task.publish(msg)	

	# builds a JSON message to start the face detection process
	def send_face_detect_command(self, command):
		# complete GO command
		self.face_detect_command["time"]["t"] = rospy.get_time()
		
		if command == "on" or command == "off":
			self.face_detect_command["data"]["task_command"] = command
		else:
			rospy.loginfo("[tunstall_manager_node] Face detect command not valid")
		
		# create message and publish it
		msg = KeyValue()
		msg.key = "interventions/famd/INFO"
		msg.value = json.dumps(self.face_detect_command)
		self.pub_task.publish(msg)	

	# builds a JSON message to be sent to the robot for waiting for some time
	def send_wait_command(self):
		# complete GO command
		self.wait_command["time"]["t"] = rospy.get_time()
		
		# create message and publish it
		msg = KeyValue()
		msg.key = "interventions/famd/INFO"
		msg.value = json.dumps(self.wait_command)
		self.pub_task.publish(msg)			
				
	# check if the type of the sensor with the provided ID is correct 					
	def check_type(self, id, in_type):
		for sensor in self.sensor_db_dict['sensor_db']:
			if int(id) == sensor['id']:
				return sensor['type'] == in_type

	# add an event to the sensor history	
	def add_to_history(self, aux, timestamp, id):

		encontrado = False
		for sensor in self.sensor_db_dict['sensor_db']:
			if int(id) == sensor['id']:
				sensor['status'].append([aux,timestamp])
				encontrado = True
				if self.verbose:
					# for debugging
					print(sensor) 
				break
						
		if not encontrado:
			rospy.logwarn("[tunstall_manager_node] WARNING: id not found")

	# load 'go' command from file
	def load_move_command_from_file(self):
		json_file = open(self.move_command_file,"r")
		self.move_command = json.load(json_file)
		json_file.close()

	# load 'speak' command from file
	def load_speak_command_from_file(self):
		json_file = open(self.speak_command_file,"r")
		self.speak_command = json.load(json_file)
		json_file.close()

	def load_face_detect_command_from_file(self):
		json_file = open(self.face_detect_command_file,"r")
		self.face_detect_command = json.load(json_file)
		json_file.close()

	def load_wait_command_from_file(self):
		json_file = open(self.wait_command_file,"r")
		self.wait_command = json.load(json_file)
		json_file.close()

	# print all sensor information to console
	def print_all_sensors(self):
		for sensor in self.sensor_db_dict['sensor_db']:
			print(sensor)

	# load sensor information from JSON file (node parameter)
	def load_sensors_from_file(self):
		json_file = open(self.sensor_file,"r")
		self.sensor_db_dict = json.load(json_file)
		json_file.close()

		# convert every sensor type from string to TSensorType
		for sensor in self.sensor_db_dict['sensor_db']:
			self.string_to_other(sensor)

		# for debug: create a fake timestamp with the current one
		# chairs = lambda x : x["type"] == TSensorType.CHAIR
		# for sensor in filter(chairs, self.sensor_db_dict['sensor_db']):
		# 	sensor["status"][-1][1] = rospy.get_time()

		# and print them
		self.print_all_sensors()
			
	# change sensor type from STRING to ENUM
	def string_to_other(self,sensor):
		if sensor['type'] == 'CHAIR' : 
			sensor['type'] = TSensorType.CHAIR
		elif sensor['type'] == 'DOOR':
			sensor['type'] = TSensorType.DOOR
		elif sensor['type'] == 'PIR':
			sensor['type'] = TSensorType.PIR
		else:
			sensor['type'] = TSensorType.UNKNOWN

	# change sensor type from ENUM to STRING
	def other_to_string(self, sensor):
		if sensor['type'] == TSensorType.CHAIR : 
			sensor['type'] = 'CHAIR'
		elif sensor['type'] == TSensorType.DOOR:
			sensor['type'] = 'DOOR'
		elif sensor['type'] == TSensorType.PIR:
			sensor['type'] = 'PIR'
		elif sensor['type'] == TSensorType.UNKNOWN:
			sensor['type'] = 'UNKNOWN'

	# save complete DB to file
	def save_db(self):
		json_file = open(self.sensor_file,"w")
		for sensor in self.sensor_db_dict['sensor_db']:
			# for debugging
			print(sensor) 
			self.other_to_string(sensor)
			print(sensor) # for debug
		json.dump(self.sensor_db_dict, json_file)
		json_file.close()

	def find_sensor_by_id(self, id):
		for sensor in self.sensor_db_dict['sensor_db']:
			if sensor["id"] == int(id):
				return sensor
		
		rospy.loginfo("[tunstall_manager_node] ERROR: id not found")
		return None

	def get_time_zone(self):

		t = time.localtime()
		current_time = time.strftime("%H:%M:%S", t)
		rospy.loginfo("[tunstall_manager_node] Current Time = " + current_time)

		return current_time


# ENTRY POINT
def main(args):
  rospy.init_node('tunstall_manager_node', anonymous=True)
  node = tunstall_manager_node()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
