# Task Coordinator
This node is in charge of interfacing the robot with the external world, that is, handling requests and sending back information to external applications like virtual caregivers, teleoperation applications, and so on.
In the context of the MoveCare project, this node is in charge of handling all intervention, alert and teleoperation requests, and commanding to the robot accordingly via de TaskManager node.

In a nutshell, the main tasks of this node are:
* To subscribe to the /mqtt2ros topic (MQTT is the default communication protocol with external apps).
* To parse the *Json* input_messages.
* To call the *Task_manager* service *add_new_task* accordingly, and configure the robot to fulfill the request.
* To forward the response back to the user/VC (after executting the request) via *MQTT*, again parsed as a *Json* msg.

Furthermore, as a side contribution, this node is in charge of estimating the ratio of success/failure of the different tasks executted by the robot.
To keep it for different runs, it save/loads resutls from file.

# Parameters:
**verbose**: To enable/disable verbose

**input_topic**: The topic to listen for new requests (default is /mqtt2ros)

**output_topic**: The topic where to publish the responses (default is /ros2mqtt)

**intervention_subtopic**: Name of the MQTT sub-topic used for interventions (see MoveCare MQTT config)

**alert_subtopic**: Name of the MQTT sub-topic used for alerts (see MoveCare MQTT config)

**user_location_subtopic**: Name of the MQTT sub-topic used for handling the most probable user location (see MoveCare MQTT config) 

**ack_subtopic**: Name of the MQTT sub-topic used for acknowledgements (see MoveCare MQTT config)

**task_statistics_file_path**: Path to file where to keep statistics of ussage/failure/sucess of the different tasks.

**user_name**: Name of the main User in the robot environment (for HRI purposes)