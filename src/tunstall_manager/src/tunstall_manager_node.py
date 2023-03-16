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
		self.scenario = "None"
		self.time_threshold = 1 # hours

		# activate timer for each 5 minutes
		rospy.Timer(rospy.Duration(0.5*60), self.timer_callback)
		
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
		# 		type : TSensorType.CHAIR,
		# 		status : False
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
	def timer_callback(self, event):
		rospy.loginfo("[tunstall_manager_node] WATCHDOG triggered")
		
		""" if self.db_counter == 0:
			self.send_face_detect_command("on")
		elif self.db_counter == 1:
			self.send_move_command("paco")
		elif self.db_counter == 2:
			self.send_speak_command("HOLA")
		elif self.db_counter == 3:
			self.send_wait_command()
		
		self.db_counter += 1 """

		# check time for all CHAIR sensors and trigger alarm if needed
		for sensor in self.sensor_db_dict['sensor_db']:
			if sensor["type"] == TSensorType.CHAIR:
				first_room = sensor["room"]
				print ("first room = "+ sensor["name"])
				print(sensor["status"][-1][0])
				is_active = sensor["status"][-1][0]
				time_difference = (rospy.get_time()-sensor["status"][-1][1])/3600
				# print(sensor["id"], "is_active", is_active, "time_dif", time_difference)
				print(f'[{sensor["id"]}] = {is_active} for {time_difference}')
				if is_active and (time_difference > self.time_threshold):
					print("se ha detectado un uso prolongado de la silla")
					# this has been sitting for more than one hour
					# TODO: Check if navigation is possible
					
					for sensor in self.sensor_db_dict['sensor_db']:
						if sensor["type"] == TSensorType.DOOR and sensor["room"] == first_room:
							my_door = sensor
							found = True
							break

					if found:	

						# puerta abierta
						if my_door["status"][-1][0] == False:
							print ("puerta abierta, dirigiéndose al lugar")
							self.send_move_command(first_room)
							self.send_speak_command("Llevas una hora sentado en la silla. ¿Por qué no sales y te tomas un descanso?")
							rospy.loginfo("[tunstall_manager_node] TIMER: more than " + str(self.time_threshold) + " hour has passed")
							self.scenario = "CANSANCIO"
						else:
							#puerta cerrada
							print("puerta cerrada, necesito que me abran la puerta")
							self.send_speak_command("Necesito que me abran la puerta")
							self.send_wait_command()
							self.scenario = "PUERTA CERRADA"
							rospy.sleep(20)

							for sensor in self.sensor_db_dict['sensor_db']:
								if sensor["type"] == TSensorType.DOOR and sensor["room"] == first_room and sensor["status"][-1][0] == False:
									print ("gracias por abrirme la puerta")	
									self.send_move_command(sensor["room"])
									self.send_speak_command("Llevas una hora sentado en la silla. ¿Por qué no sales y te tomas un descanso?")
									rospy.loginfo("[tunstall_manager_node] TIMER: more than " + str(self.time_threshold) + " hour has passed")
									self.scenario = "CANSANCIO"
								else :
									print ("la puerta sigue cerrada")

								return
								
						

							#return			


	# process face recognition result
	def face_recognized_callback(self,msg):

		if self.verbose:
			print(f'Nombre reconocido: {msg.data}')

		text_to_say = ""
		#if self.scenario == "CAFE":
			## callback from the topic "nombre" 
		if str(msg.data) == "desconocido":
			text_to_say = "Ese no es su asiento, levántese, por favor"

		elif str(msg.data) == "Roberto":
			text_to_say = "tu café es el de la izquierda, sin azucar"

		elif str(msg.data) == "Paco":
			text_to_say = "Tu café es el de la derecha, con leche sin lactosa"

		else: 
			text_to_say = "Lo siento, no traigo café para usted"


		#elif self.scenario == "CANSANCIO":
		#	text_to_say = "Deberías tomarte un descanso, " + str(msg.data)

		#else:
		#	print(text_to_say)
		#	text_to_say = "No hay escenario para esto, " + str(msg.data)

		# json to say something
		print(text_to_say)
		self.send_speak_command(text_to_say)
	
	
	# a sensor has been triggered
	def tunstall_trigger_callback(self,msg):
		if self.verbose:
			print("Received tunstall trigger")

		if not self.active:
			return

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

		#TODO: check why this does not work
		#if self.verbose:
		#	rospy.loginfo("[tunstall_manager_node] Code Received: ",str(code))
		#	rospy.loginfo("[tunstall_manager_node] Date Received: ",date_time_str)
			
		date_time_obj = datetime.datetime.strptime(date_time_str, "%Y-%m-%d %H:%M:%S.%f")
		timestamp = to_seconds(date_time_obj)
		#timestamp = datetime.timestamp(date_time_obj)

		# SCENARIOS:
		# -- [1] CAFE: A chair is activated, the robot goes there with two cups, recognizes the person and offers their coffee.
		# -- [2] CANSANCIO: Every 5 minutes, the node checks if any chair has been activated for more than one hour. If so, goes to ask him to get some exercise. (No face recognition) ALthough it could use face recognition to say de person's
		# -- [3] SECURITY: All chairs are activated, the main door is open and the PIR is activated. The robot goes to the PIR, recognizes the face and ask for security if needed.
		# -- Secondary: in all scenarios, if the robot needs to traverse a closed door, it asks for help, waits some time and if nobody opens it then cancels the mission.
		
		# a door has been open
		if code == "AR" : 
			if self.check_type(id,TSensorType.DOOR):
				aux = False
				self.add_to_history(aux, timestamp, id)
			else:
				print("Warning: This kind of sensor does not use this code")

		# a door has been closed
		elif code == "AQ" : 
			if self.check_type(id,TSensorType.DOOR):
				aux = True
				self.add_to_history(aux, timestamp, id)
			else:
				print("Warning: This kind of sensor does not use this code")
		
		# someone has sit
		elif code == "BA" : 
			if self.check_type(id,TSensorType.CHAIR):
				aux = True
				self.add_to_history(aux, timestamp, id)
			else:
				print("Warning: This kind of sensor does not use this code")

		# someone has stood up
		elif code == "AZ" : 
			if self.check_type(id,TSensorType.CHAIR):
				aux = False
				self.add_to_history(aux, timestamp, id)
			else:
				print("Warning: This kind of sensor does not use this code")
			
		# someone has passed the way: PIR activated
		elif code == "BH" : 
			if self.check_type(id,TSensorType.PIR):
				print("BH triggered, moving")
				aux = True
				self.add_to_history(aux, timestamp, id)
				# self.send_move_command("Alberto")
				# self.send_speak_command("Hola ke ase")
				# self.send_move_command("laboratorio1")
				# self.send_wait_command()
				
				# self.scenario = "SEGURIDAD"
				# self.send_move_command("PIR")
				self.send_face_detect_command("on")
				
			else:
				print("Warning: This kind of sensor does not use this code")



		# if the chair sensor is activated go there and say something
		if code == "BA":
			if self.verbose:
				print("Se ha activado sensor de silla")			
			
			if id == "03":
				desired_id = "03"
				target = "Paco"
				if self.verbose:
					print("La id es la de la silla de Paco")

				linked_door_id = "01"
				linked_door_sensor = self.find_sensor_by_id(linked_door_id)
				#print(linked_door_sensor["status"][-1][0])
				is_closed = linked_door_sensor["status"][-1][0]
				


			elif id == "02":
				desired_id = "02"
				target = "Roberto"
				if self.verbose:
					print("La id es la de la silla de Roberto")
					
				linked_door_id = "01"
				linked_door_sensor = self.find_sensor_by_id(linked_door_id)
				#print(linked_door_sensor["status"][-1][0])
				is_closed = linked_door_sensor["status"][-1][0]
				
					

			# check if the door is open or closed

			if is_closed == False:
				if self.verbose:
						print("puerta abierta")
						## go to to the position of the chair sensor with some correction
						self.send_move_command(target)							
						if self.verbose:
							print("Moviendose a la silla de "+target)

						## activate face_detection_node 
						# json message to activate face_detection_node
						self.send_face_detect_command("on")
						if self.verbose:
							print("detectando cara...")

						## activate speak node
						# json message to activate speak
						# self.face_recognized_callback(msg)	

						#self.scenario = "CAFE"

						
			else:
				self.scenario = "PUERTA BLOQUEA EL PASO"
				if self. verbose :
					print("puerta cerrada")
				## activate speak node
				# json message to activate sepak_node
				self.send_speak_command("Necesito que me abran la puerta")
				if self. verbose :
					print ("Necesito que me abran la puerta")
				self.send_wait_command()
				self.scenario = "PUERTA CERRADA"
				rospy.sleep(20)

				if linked_door_sensor["status"[-1][0]] == False:
					if self.verbose:
							print("puerta abierta")
					## go to to the position of the chair sensor with some correction
							self.send_move_command(target)							
							if self.verbose:
								print("Moviendose a la silla de "+target)

							## activate face_detection_node 
							# json message to activate face_detection_node
							self.send_face_detect_command("on")
							if self.verbose:
								print("detectando cara...")

							## activate speak node
							# json message to activate speak
							self.face_recognized_callback(msg)		

							#self.scenario = "CAFE"
				else :
							print ("la puerta sigue cerrada")

							return

					
							

			# if the robot is neither in "CANSANCIO" scenario nor in "PUERTA BLOQUEA EL PASO" scenario
			# check if it is in "C"
			#if self.scenario != "CANSANCIO" and self.scenario != "PUERTA BLOQUEA EL PASO":
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
			#			self.scenario = "CAFE"

				
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
				print("sensor['type']: ", sensor['type'] )
				return sensor['type'] == in_type
		#return False


	# add an event to the sensor history	
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
