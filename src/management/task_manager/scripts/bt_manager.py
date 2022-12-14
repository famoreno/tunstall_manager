#!/usr/bin/env python3

import rospy
from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *
from pi_trees_gui.pi_trees_gui import *
from bt_task_lib import *       # Import library with avialbale TASKS created as BehaviourTrees (subtrees)
from task_manager.srv import *
import numpy as np


class bt_manager:
    
    # VARS of the class
    bt_root = None              # Root of the BT
    bt_manager_gui = None       # Handler of the GUI for the BT
    bt_task_counter = 0         # Counter to assign unique task_id

    # ---------------------------------------------------------------------
    #                           INIT (MAIN)
    # ---------------------------------------------------------------------
    def __init__(self):
        rospy.init_node("bt_manager")
        self.verbose = rospy.get_param("~verbose",False)
        self.hasGUI = rospy.get_param("~gui",False)
        self.main_battery = rospy.get_param("~battery_topic","giraff_node/battery")

        # Read global_param /user_language
        has_user_language = True
        while (not rospy.is_shutdown()) and (not has_user_language):
            if rospy.has_param("/user_language"):
                self.main_language = rospy.get_param("/user_language","EN")
                rospy.loginfo("\n\n[TaskManager] user_language parameter set to %s.\n\n", self.main_language);
                has_user_language = True
            else:
                rospy.logwarn("[TaskManager] Waiting for user_language parameter to be set.");
                rospy.sleep(1.0);

        # 0. Create the root node (Sequence) and the GUI
        # -----------------------------------------------
        self.bt_root = PrioritySequence("ROOT", reset_after=False, announce=False)
        if self.hasGUI:
            self.bt_manager_gui = bt_gui("BT-GUI")      # Creates a GUI with a Text widget
        # Set the shutdown function
        rospy.on_shutdown(self.shutdown)


        # SERVICES (name, type (file.srv), handler)
        # -----------------------------------------
        # Services to add/remove tasks from other ROSpkgs
        rospy.Service('~add_new_task', addTask, self.add_new_task_from_service_request)
        rospy.Service('~remove_task', removeTask, self.remove_task_from_service_request)
        # Service to evaluate the cost of executing a Task (sub-tree)
        rospy.Service('~evaluate_task', evaluateTask, self.evaluate_task_from_service_request)
        # Topic to publish Task results
        self.task_result_pub = rospy.Publisher('/bt_manager/task_manager_result', String, queue_size=10)
        self.tree_pub = rospy.Publisher('/bt_manager/tree', String, queue_size=10)
        self.current_task_pub = rospy.Publisher('/bt_manager/current_task', String, queue_size=1)
        self.current_task = ""

        # Any initial Task to launch? (from launch file)
        # ------------------------------------------------
        if self.verbose:
            print (colored("[bt_manager] INITIAL TASKS:", "green"))
        moreTasks = True
        i = 0
        while moreTasks:
            if rospy.has_param("~task_" + str(i) + "/name"):
                self.load_initial_task(i)
                i += 1
            else:
                moreTasks = False


        # RUN: Run the tree (in a loop)
        # --------------------------------
        while not rospy.is_shutdown():
            # spin_once() --> in python there is no need to spin_once (they are different threads)
            self.remove_finished_tasks()         # Finished Tasks?
            self.reorder_tasks_by_priority()     # Un-ordered Tasks?
            rospy.sleep(0.1)

            # Publish tree for visualization
            self.publish_tree(self.bt_root)                                     # publish topic (serialization)

            # Publish current task in exection (Running)
            running_task = self.get_running_task()
            if running_task is not self.current_task:
                self.current_task_pub.publish(running_task)
                self.current_task = running_task


            # RUN the Tree
            self.bt_root.status = self.bt_root.run(min_priority=-1)             # Execute BT
            self.bt_root.task_finished = False                                  # Allow execution again and again

            if self.hasGUI:
                self.bt_manager_gui.updateGui(self.bt_root, use_colors=True)    # GUI


            # TODO --> run in a different thread




    # =============================================================
    # ==================== GET RUNNING TASK  ======================
    # =============================================================
    def get_running_task(self):
        if len(self.bt_root.children) == 0:         # No task planned
            return "IDLE"
        else:
            # Return first running task by priority
            for c in self.bt_root.children:
                #if c.status is TaskStatus.RUNNING:
                return c.name
            # Reaching this point means all tasks are done, so lets return IDLE again
            #print colored("[TaskManager] Error returing Current Running Task.","red")
            return "IDLE"


    # =============================================================
    # ====================  ADD TASK SRV  =========================
    # =============================================================
    def add_new_task_from_service_request(self,req):
        """
        Load a new task (from service call)
        :param req: The name, priority, permanence and args of the task to create
        :return: int task_id
        """
        task_name = req.task_name
        task_prior = req.task_priority
        task_perm = req.task_permanence
        task_args = req.task_args       # list of strings.. (string[] in the srv)
        task = None
        task_tree = None

        if task_name == "shutdown":
            # task_args = []
            # Create Task (subtree of Tasks)
            task = subtree_shutdown()
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "say":
            # task_args = ["text to say"]
            dialogue = task_args[0]
            # Create Task (subtree of Tasks)
            task = subtree_say(dialogue)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "rem":
            # task_args = [hri_code, user_name, rem_code]
            hricode = int(task_args[0])     # int
            user_name = task_args[1]        # string
            remcode = [task_args[2]]        # string list

            # Create Task (subtree of Tasks)
            task = subtree_hri(hricode, user_name, remcode)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "demo":    
            # Create Task (subtree of Tasks)
            task = subtree_demo()
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "face_detection":
            # task_args = ["action_type", "time_out", "just_once", "speech"]
            ac_type = int(task_args[0])
            tout = float(task_args[1])
            just_1 = task_args[2] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            speech = task_args[3] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            # Create Task (subtree)
            task = subtree_face_detection(ac_type, tout, just_1, speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)
                
        elif task_name == "find_person":
            # task_args = ["action_type", "time_out", "just_once", "speech"]
            ac_type = int(task_args[0])
            tout = float(task_args[1])
            just_1 = task_args[2] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            speech = task_args[3] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            # Create Task (subtree)
            task = subtree_find_person(ac_type, tout, just_1, speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "go_to_point":
            #task_args = ["goal_label", "[x, y, phi]"]
            goal_name = "GO_TO_" + task_args[0]
            pose2D = ast.literal_eval(task_args[1])     #list [x, y , phi]
            quat = quaternion_from_euler(0, 0, pose2D[2])
            pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))

            # Create Task (subtree)
            task = subtree_go_to_point(goal_name, pose3D)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "wait":
            # task_args = ["time_sec to wait"]
            time_sec = float(task_args[0])
            # Create Task (subtree)
            task = subtree_wait(time_sec)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "dock":
            # task_args = [docking_pose, speech]
            pose2D = ast.literal_eval(task_args[0])  # list [x, y , phi]
            quat = quaternion_from_euler(0, 0, pose2D[2])
            pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
            speech = task_args[1] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            # Create Task (subtree)
            task = subtree_dock(self.main_battery, pose3D, speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "undock":
            # task_args = [speech]
            speech = task_args[0] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            # Create Task (subtree)
            task = subtree_undock(speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        # Not-recommended (use instead the battery_manager node)
        elif task_name == "battery_manager":
            # task_args = ["bat_topic","low_bat_th","critical_bat_th","[x, y, phi]"]
            battery_topic = task_args[0]
            low_battery_threshold = float(task_args[1])         # Low-Battery threshold (V) -> To recommend Recharge!
            critical_battery_threshold = float(task_args[2])    # Critial-Battery threshold (V) -> To command Recharge!
            dock_pose2D = ast.literal_eval(task_args[3])        # list [x, y , phi]
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            # Create Task (subtree)
            task = subtree_battery_manager(battery_topic, low_battery_threshold, critical_battery_threshold,
                                           docking_station_pose, True)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)
            
        elif task_name == "battery_manager_simul":
            # task_args = ["bat_topic","bat_th","[x, y, phi]"]
            battery_topic = task_args[0]
            low_battery_threshold = float(task_args[1])   # Low-Battery threshold (V) -> To command Recharge!
            dock_pose2D = ast.literal_eval(task_args[2])  # list [x, y , phi]
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            # Create Task (subtree)
            task = subtree_battery_manager_simul(battery_topic, low_battery_threshold, docking_station_pose)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "patrol":
            # task_args = ["random", "speech","pose1", "pose2", ..., "poseN"]
            random = task_args[0] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            speech = task_args[1] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            num_poses = len(task_args)-2
            self.waypoints = []
            for i in range(2,num_poses+2):
                pose2D = ast.literal_eval(task_args[i])     # list [x, y , phi]
                quat = quaternion_from_euler(0, 0, pose2D[2])
                pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
                self.waypoints.append(pose3D)
            # Create Task (subtree)
            task = subtree_patrol(self.waypoints, random=random, battery_topic=self.main_battery, speech=speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "gsl":
            # task_args = ["gas_conc_topic", "gas_conc_threshold", gsl_method]
            gas_conc_topic = task_args[0]               # gas sensor topic to monitor gas concentratoin
            gas_conc_threshold = float(task_args[1])    # th to rise the search!
            gsl_method = task_args[2]                   # method to use: smenatics, cfd, plume_tracking, etc

            # Create Task
            task = subtree_gsl(gas_conc_topic=gas_conc_topic, gas_conc_threshold=gas_conc_threshold,
                               gsl_method=gsl_method)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "cmd_vel_time":
            # Load additional params of this task
            lin_speed = float(task_args[0])
            ang_speed = float(task_args[1])
            time_sec = float(task_args[2])
            # Create Task (subtree)
            task = subtree_cmd_vel_time(lin_speed,ang_speed, time_sec)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "movecare_intervention":
            # Interventions include: WM, CT, CA, SQ
            # params(list): userName(string), HRIcode(int), numRooms(int), roomLabels(string[]), roomPoses(string[]), [texts](string list)

            # Username
            username = task_args[0]
            # HRI code
            hri_code = int(task_args[1])
            # num Rooms/Poses in the environment
            num_poses = int(task_args[2])
            # Pose and Labels of locations in the environment
            labels = task_args[3:3+num_poses]                             # string list
            poses = task_args[3+num_poses:3+2*num_poses]                  # string list
            locationsArray = PoseArray()
            locationsArray.header.frame_id = "map"
            locationsArray.header.stamp = rospy.Time.now()
            #print colored("[bt_manager] MoveCare intervention: num_poses=" + str(num_poses) + " size=" + str(len(task_args)), "green")
            
            for i in range(0,num_poses):
                pose2D = ast.literal_eval(poses[i])     # list [x, y, phi]
                quat = quaternion_from_euler(0, 0, pose2D[2])
                pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
                locationsArray.poses.append(pose3D)

            # Do we have Additional Parameters? (May be more than one string)
            if len(task_args) > 3+2*num_poses:
                additional_p = task_args[3+2*num_poses:]  # string list
            else:
                additional_p = []                         # string list


            # Create Task (subtree)
            task = subtree_movecare_intervention(self.main_battery, self.main_language, username, hri_code, labels, locationsArray, additional_p)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)


        elif task_name == "movecare_rfid_search":
            # params: objectID, location1, location2, etc...
            objectID = task_args[0]     #string

            locationsArray = PoseArray()
            locationsArray.header.frame_id = "map"
            locationsArray.header.stamp = rospy.Time.now()
            num_poses = len(task_args)-1
            for i in range(1,num_poses+1):
                pose2D = ast.literal_eval(task_args[i])     # list [x, y , phi]
                quat = quaternion_from_euler(0, 0, pose2D[2])
                pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
                locationsArray.poses.append(pose3D)

            # Create Task (subtree)
            task = subtree_movecare_rfid_search(objectID, locationsArray, self.main_language, self.main_battery)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "recharge_till_full":
            # params: docking_station_pose, speech
            dock_pose2D = ast.literal_eval(task_args[0])        # list [x, y , phi]
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            speech = task_args[1] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            # Create Task (subtree)
            task = subtree_recharge_till_full(self.main_battery, docking_station_pose, speech)

            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "idle_to_dock":
            # params: docking_station_pose, idle_timeout, speech
            dock_pose2D = ast.literal_eval(task_args[0])        # list [x, y , phi]
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            idle_timeout = float(task_args[1])
            speech = task_args[2] in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']
            # Create Task (subtree)
            task = subtree_idle_to_dock(self.main_battery, docking_station_pose, idle_timeout, speech)

            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)


        elif task_name == "teleoperation":
            # Input params: CallerID, caller_name, confirmation_url, Teleop_URL, Request_Confirmation, num_Locations, locationLabels, locationPoses):
            callerID = task_args[0]
            callerName = task_args[1]
            conf_url = task_args[2]
            teleop_url = task_args[3]
            req_conf = task_args[4] in ['true', 'True', '1', 'yes', 'Yes', 'yup', 'certainly']
            # num Rooms/Poses in the environment
            num_poses = int(task_args[5])

            # Pose and Labels of locations in the environment
            labels = task_args[6:6+num_poses]                             # string list
            poses = task_args[6+num_poses:6+2*num_poses]                  # string list
            locationsArray = PoseArray()
            locationsArray.header.frame_id = "map"
            locationsArray.header.stamp = rospy.Time.now()

            for i in range(0,num_poses):
                pose2D = ast.literal_eval(poses[i])     # list [x, y, phi]
                quat = quaternion_from_euler(0, 0, pose2D[2])
                pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
                locationsArray.poses.append(pose3D)



            # Create Task (subtree)
            task = subtree_teleoperation(callerID, callerName, conf_url, teleop_url, req_conf, self.main_battery, labels, locationsArray)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

            # TFG APP ANDROID
        elif task_name == "surveillance":
            # task_args = ["boolean to start/stop"]
            state = task_args[0]

            # Create Task (subtree of Tasks)
            task = subtree_surveillance(state) #Añadir en bt_task_lib

            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        
        elif task_name == "get_image":
            # task_args = ["string with the command 'get_image'"]
            command = task_args[0]

            # Create Task (subtree of Tasks)
            task = subtree_get_image(command) #Añadir en bt_task_lib

            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "tunstall_manager":
            # Load additional params of this task
            message = task_args[0]

            # Create Task (subtree)
            task = subtree_tunstall_manager(message)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)

        elif task_name == "face_detection_tfg":
            # task_args = ["boolean to start/stop"]
            state = task_args[0]

            # Create Task (subtree of Tasks)
            task = subtree_face_detection_tfg(state) # Añadir en bt_task_lib

            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree, req.task_impact)                

        else:
            print(colored("[bt_manager] ERROR creating new Task (srv): Unknown New Task... Doing nothing!", "red"))
            return -1

        # Service response: Task_id or -1 if error
        # -------------------------------------------
        if task.task_created is True:
            return task.ROOT.id
        else:
            return -1


    # =============================================================
    # ====================  REMOVE TASK SRV  ======================
    # =============================================================
    def remove_task_from_service_request(self, req):
        """
        Try to remove an existing task given its ID
        If ID = -1, removes all tasks!
        :param req.task_id: The task id (int8)
        :param req.info: Description to remove the task (string)
        :return: None
        """
        if req.task_id == -1:
            self.remove_all_tasks(req.info)
            return True
        else:
            return self.remove_task_by_id(req.task_id, req.info)


    # =============================================================
    # ==================  EVALUATE TASK SRV  ======================
    # =============================================================
    def evaluate_task_from_service_request(self, req):
        """
        Evaluate the cost of running a task (subtree). Useful for planning
        :param req: The task name (task type) and the task arguments
        :return: res: task_evaluation (json)
        """
        return self.evaluate_task(req.task_name, req.task_args)



    # =============================================================
    # ====================  LOAD INITIAL TASKS  ===================
    # =============================================================
    def load_initial_task(self, i):
        """
        Load initial tasks from launch file
        :param i: id of the task to load task_0, task_1...(see yaml file)
        :return: None
        """
        task_name = rospy.get_param("~task_" + str(i) + "/name", "UnnameTask")
        task_prior = rospy.get_param("~task_" + str(i) + "/priority", 1)
        task_perm = rospy.get_param("~task_" + str(i) + "/permanence", False)

        if task_name == "say":
            # Load additional params of this task
            dialogue = rospy.get_param("~task_" + str(i) + "/text_to_say", "Buenos dias, inicializando task manager")
            # Create Task (subtree)
            task = subtree_say(dialogue)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "rem":
            # task_args = [hri_code, user_name, rem_code]
            hricode = int(rospy.get_param("~task_" + str(i) + "/hricode", 7))           # int
            remcode = [rospy.get_param("~task_" + str(i) + "/remcode", "REM13")]        # string list

            # Create Task (subtree of Tasks)
            task = subtree_hri(hricode, "", remcode)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)


        elif task_name == "face_detection":
            # Load additional params of this task
            ac_type = rospy.get_param("~task_" + str(i) + "/action_type", 0)
            tout = rospy.get_param("~task_" + str(i) + "/timeout", 5.0)
            just_1 = rospy.get_param("~task_" + str(i) + "/just_once", False )
            speech = rospy.get_param("~task_" + str(i) + "/speech", False )
            # Create Task (subtree)
            task = subtree_face_detection(ac_type, tout, just_1, speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)
                
        elif task_name == "find_person":
            # Load additional params of this task
            ac_type = rospy.get_param("~task_" + str(i) + "/action_type", 0)
            tout = rospy.get_param("~task_" + str(i) + "/timeout", 5.0)
            just_1 = rospy.get_param("~task_" + str(i) + "/just_once", False )
            speech = rospy.get_param("~task_" + str(i) + "/speech", False )
            # Create Task (subtree)
            task = subtree_find_person(ac_type, tout, just_1, speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)
	
        elif task_name == "go_to_point":
            # Load additional params of this task
            goal_name = "GO_TO_" + rospy.get_param("~task_" + str(i) + "/goal_label", "POINT")
            pose2D = rospy.get_param("~task_" + str(i) + "/pose", [])  # list 2Dpose [x, y , phi]
            quat = quaternion_from_euler(0, 0, pose2D[2])
            pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
            # Create Task (subtree)
            task = subtree_go_to_point(goal_name, pose3D)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "wait":
            # Load additional params of this task
            time_sec = rospy.get_param("~task_" + str(i) + "/time_sec", [])
            # Create Task (subtree)
            task = subtree_wait(time_sec)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)
                

        elif task_name == "dock":
            # task_args = [docking_pose, speech]
            pose2D = rospy.get_param("~task_" + str(i) + "/docking_pose", [])  # list 2Dpose [x, y , phi]
            quat = quaternion_from_euler(0, 0, pose2D[2])
            pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
            speech = rospy.get_param("~task_" + str(i) + "/speech", False)
            # Create Task (subtree)
            task = subtree_dock(pose3D, speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "undock":
            # task_args = [speech]
            speech = rospy.get_param("~task_" + str(i) + "/speech", False)
            # Create Task (subtree)
            task = subtree_undock(speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        # Non-recommended (use instead the battery_manager node)
        elif task_name == "battery_manager":
            # Load additional params of this task
            battery_topic = rospy.get_param("~task_" + str(i) + "/battery_topic", "battery")
            low_battery_threshold = rospy.get_param("~task_" + str(i) + "/low_battery_threshold", "24.0")
            critical_battery_threshold = rospy.get_param("~task_" + str(i) + "/critical_battery_threshold", "22.0")
            dock_pose2D = rospy.get_param("~task_" + str(i) + "/docking_station_pose", [])   # list 2Dpose [x, y , phi]
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            # Create Task (subtree)
            task = subtree_battery_manager(battery_topic, low_battery_threshold, critical_battery_threshold,
                                           docking_station_pose, True)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "battery_manager_simul":
            # Load additional params of this task
            battery_topic = rospy.get_param("~task_" + str(i) + "/battery_topic", "battery")
            low_battery_threshold = rospy.get_param("~task_" + str(i) + "/low_battery_threshold", "12.0")   # Low-Battery threshold (V) -> To command Recharge!
            dock_pose2D = rospy.get_param("~task_" + str(i) + "/docking_station_pose", [])                  # list 2Dpose [x, y , phi]
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            # Create Task (subtree)
            task = subtree_battery_manager_simul(battery_topic, low_battery_threshold, docking_station_pose)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "patrol":
            # Load additional params of this task (poses to patrol)
            random = rospy.get_param("~task_" + str(i) + "/random", False)
            speech = rospy.get_param("~task_" + str(i) + "/speech", False)
            more_poses = True
            pose_number = 0
            self.waypoints = list()
            while more_poses:
                if rospy.has_param("~task_" + str(i) + "/pose_" + str(pose_number)):
                    pose2D = rospy.get_param("~task_" + str(i) + "/pose_" + str(pose_number), [])  # list 2Dpose [x, y , phi]
                    quat = quaternion_from_euler(0, 0, pose2D[2])
                    pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
                    self.waypoints.append(pose3D)
                    pose_number += 1
                else:
                    more_poses = False
            # Create Task (subtree)
            task = subtree_patrol(self.waypoints, random=random, battery_topic=self.main_battery, speech=speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "gsl":
            # Load additional params of this task
            gas_conc_topic = rospy.get_param("~task_" + str(i) + "/gas_conc_topic", "PID_readings")
            gas_conc_threshold = rospy.get_param("~task_" + str(i) + "/gas_conc_threshold", "10")
            gsl_method = rospy.get_param("~task_" + str(i) + "/gsl_method", "semantic")
            # Create Task (subtree)
            task = subtree_gsl(gas_conc_topic=gas_conc_topic, gas_conc_threshold=gas_conc_threshold,
                               gsl_method=gsl_method)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "cmd_vel_time":
            # Load additional params of this task
            lin_speed = rospy.get_param("~task_" + str(i) + "/lin_speed", 0.0)
            ang_speed = rospy.get_param("~task_" + str(i) + "/ang_speed", 0.0)
            time_sec = rospy.get_param("~task_" + str(i) + "/time_sec", 0.0)
            # Create Task (subtree)
            task = subtree_cmd_vel_time(lin_speed, ang_speed, time_sec)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)
                
        elif task_name == "movecare_scn":
            # COGNITIVE GAME: Load additional params of this task
            user_pose2D = rospy.get_param("~task_" + str(i) + "/user_expected_pose", [])
            quat = quaternion_from_euler(0, 0, user_pose2D[2])
            user_expected_pose = (Pose(Point(user_pose2D[0], user_pose2D[1], 0.0),
                                       Quaternion(*quat)))
            username = rospy.get_param("~task_" + str(i) + "/user_name", "Human")
            game_code = rospy.get_param("~task_" + str(i) + "/game_code", 4)
            dock_pose2D = rospy.get_param("~task_" + str(i) + "/docking_station_pose", [])
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            # Create Task (subtree)
            task = subtree_movecare(user_expected_pose, username, game_code, docking_station_pose)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)

        elif task_name == "recharge_till_full":
            # params: docking_station_pose, speech
            dock_pose2D = rospy.get_param("~task_" + str(i) + "/docking_station_pose", [])
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            speech = rospy.get_param("~task_" + str(i) + "/speech", False)

            # Create Task (subtree)
            task = subtree_recharge_till_full(self.main_battery, docking_station_pose, speech)
            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)


        elif task_name == "idle_to_dock":
            # params: docking_station_pose, idle_timeout, speech
            dock_pose2D = rospy.get_param("~task_" + str(i) + "/docking_station_pose", [])
            quat = quaternion_from_euler(0, 0, dock_pose2D[2])
            docking_station_pose = (Pose(Point(dock_pose2D[0], dock_pose2D[1], 0.0),
                                         Quaternion(*quat)))
            idle_timeout = rospy.get_param("~task_" + str(i) + "/idle_timeout", 60)
            speech = rospy.get_param("~task_" + str(i) + "/speech", False)

            # Create Task (subtree)
            task = subtree_idle_to_dock(self.main_battery, docking_station_pose, idle_timeout, speech)

            if task.task_created is True:
                task_tree = task.ROOT
                task_tree.priority = task_prior
                task_tree.permanence = task_perm
                self.insert_new_task_with_priority(task_tree)


        else:
            print(colored("ERROR: Unknown New Task... Doing nothing!", "red"))

    # =============================================================
    # ====================  INSERT TASKS IN BT  ===================
    # =============================================================
    def insert_new_task_with_priority(self, task_tree, impact="reset"):
        """
        Inserts the new task as a child of the bt_root according to its priority.
        If another task exists with the same priority, the new task is inserted after the former.
        :param task_tree: The behaviour_tree of the new task to manage
        :param impact: What is the impact over the current tasks planned with a lower priority --> none, reset, cancel
        :return: None
        """

        # Set task ID only for the root element of the task (unique identifier)
        task_tree.id = self.bt_task_counter
        self.bt_task_counter += 1

        # insert into main BT
        if len(self.bt_root.children) == 0:         # first task
            self.bt_root.add_child(task_tree)
        else:
            # Insert the task
            task_inserted = False
            for c, idx in zip(self.bt_root.children, range(len(self.bt_root.children))):
                if c.priority < task_tree.priority:
                    self.bt_root.insert_child(task_tree, idx)
                    task_inserted = True
                    break

            if task_inserted:
                if impact == "reset":
                    # Reset all tasks with lower priority than the current one
                    self.reset_tasks_by_priority(task_tree.priority)
                elif impact == "cancel":
                    # Remove all tasks with lower priority than the current one
                    self.remove_tasks_by_priority(task_tree.priority)
            else:
                # Reaching this point means new task is the lower on priority (add to the end)
                # Impact does not affect since we are the task with lower priority
                self.bt_root.add_child(task_tree)

        # Verbose
        if self.verbose:
            print (colored("[bt_manager] New Task scheduled with ID[" + str(task_tree.id) + "]: " + task_tree.name +
                          " Priority=" + str(task_tree.priority) + " permanence=" + str(task_tree.permanence), "green"))


    # =============================================================
    # ====================  REMOVE TASKS COMPLETED  ===============
    # =============================================================
    def remove_finished_tasks(self):
        """
        Inspect the current BTree and remove tasks already finished
        :return: None
        """
        task_iter = 0
        while len(self.bt_root.children) > task_iter:
            c = self.bt_root.children[task_iter]
            if (c.status is TaskStatus.SUCCESS) or (c.status is TaskStatus.FAILURE):
                # Task is finished
                if c.permanence:
                    # reset task, so we can run it again
                    print (colored("[bt_manager] Task with ID[" + str(c.id) + "]: " + c.name + " is Permanent >> Reset and continue","yellow"))
                    c.reset()
                    task_iter += 1
                else:
                    if self.verbose:
                        if (c.status is TaskStatus.SUCCESS):
                            print (colored("[bt_manager] Task with ID[" + str(c.id) + "]: " + c.name +
                                      " has finished with state SUCCESS. Trace: "+ str(c.trace) +"\n", "green"))
                        else:
                            print (colored("[bt_manager] Task with ID[" + str(c.id) + "]: " + c.name +
                                      " has finished with state FAILURE. Trace: "+ str(c.trace) +"\n", "yellow"))

                    # remove from sub-tree (reset is also necessary to stop all Action Servers that may be running)
                    c.reset()
                    self.bt_root.remove_child(c)

            elif (c.status is TaskStatus.RUNNING):
                task_iter += 1

            else:
                # Task not yet executed (do nothing)
                task_iter += 1


    # =============================================================
    # ====================  REMOVE TASK BY ID  ====================
    # =============================================================
    def remove_task_by_id(self, task_id, info):
        """
        Inspect the current BTree and remove the task with provided "id"
        :return: Bool
        """
        task_iter = 0
        while len(self.bt_root.children) > task_iter:
            c = self.bt_root.children[task_iter]
            if c.id is task_id:
                print (colored("[bt_manager] Task[" + str(task_id) + "]:" + c.name + " killed by request.", "red"))
                # Reset task before deletion to stop all Action Servers
                c.reset()
                # Force to notify result (if announce=true)
                if c.announce:
                    c.status = TaskStatus.FAILURE
                    if info == "":
                        c.trace = '"TaskManager":"Task cancelled by request"'
                    else:
                        c.trace = '"TaskManager":"' + info + '"'
                    c.announce_result()
                # remove leaf
                self.bt_root.remove_child(c)
                return True
            else:
                task_iter += 1
        # Reaching this points mean the task id has not been found!
        print (colored("[bt_manager] Task[" + str(task_id) + "]: cannot be killed by request. TaskID not found!", "red"))
        return False


    # =============================================================
    # =================  REMOVE ALL PENDING TASKS  ================
    # =============================================================
    def remove_all_tasks(self, info):
        """
        Removes all Tasks in the BTree.
        :return: None
        """
        task_iter = 0
        while len(self.bt_root.children) > task_iter:
            c = self.bt_root.children[task_iter]
            # Reset task before deletion to stop all Action Servers
            c.reset()
            # Force to notify result (if announce=true)
            if c.announce:
                c.status = TaskStatus.FAILURE
                if info == "":
                    c.trace = '"TaskManager":"Task cancelled by request"'
                else:
                    c.trace = '"TaskManager":"' + info + '"'
                c.announce_result()
            # remove leaf
            self.bt_root.remove_child(c)


    # =============================================================
    # =================  REMOVE TASK BY PRIORITY  =================
    # =============================================================
    def remove_tasks_by_priority(self, P):
        """
        Removes all Tasks in the BTree with a Priority < P
        :return: None
        """
        task_iter = 0
        while len(self.bt_root.children) > task_iter:
            c = self.bt_root.children[task_iter]
            if c.priority < P:
                print (colored("[bt_manager] Task[" + str(c.id) + "]:" + c.name + " killed by request.", "red"))
                # Reset task before deletion to stop all Action Servers
                c.reset()

                # reset() does not finish the task (it is still running)
                # Force to notify result (if announce=true) as we are killing the task
                if c.announce:
                    c.status = TaskStatus.FAILURE
                    c.trace = '"TaskManager":"Task cancelled by request"'
                    c.announce_result()
                # remove leaf
                self.bt_root.remove_child(c)
            else:
                task_iter += 1

    # =============================================================
    # =================  RESET TASK BY PRIORITY  =================
    # =============================================================
    def reset_tasks_by_priority(self, P):
        """
        Reset all Tasks in the BTree with a Priority < P
        :return: None
        """
        task_iter = 0
        while len(self.bt_root.children) > task_iter:
            c = self.bt_root.children[task_iter]
            if c.priority < P:
                print (colored("[bt_manager] Task[" + str(c.id) + "]:" + c.name + " reset by request.", "red"))
                # Reset task (usefull to stop all Action Servers)
                c.reset()
            # next subtree
            task_iter += 1


    # =============================================================
    # ====================  REORDER TASKS   =======================
    # =============================================================
    def reorder_tasks_by_priority(self):
        """
        Inspect the current BTree and reorder tasks by priority
        Useful for tasks with Dynamic priority (see recharge_till_full)
        :return: None
        """

        self.bt_root.children = sorted(self.bt_root.children, key=lambda t: t.priority, reverse=True)   # sort by priority


    # =============================================================
    # ====================  EVALUATE TASK   =======================
    # =============================================================
    def evaluate_task(self, task_name, task_args):
        return "{\"execution_time_sec\":\"24\",\"success_prob\":\"0.8\"}"


    # =============================================================
    # ==========================  SHUTDOWN  =======================
    # =============================================================
    def shutdown(self):
        rospy.loginfo("[bt_manager] Cancelling pending tasks...")
        # self.move_base.cancel_all_goals()
        # self.cmd_vel_pub.publish(Twist())
        self.remove_all_tasks("System shutdown detected")
        rospy.loginfo("[bt_manager] Closing GUI...")
        if self.hasGUI:
            self.bt_manager_gui.shutdown()
        rospy.sleep(0.1)


    # =============================================================
    # ====================  PUBLISH TREE JSON  ====================
    # =============================================================
    def publish_tree(self, tree):
        """
            A serialization of the task tree in JSON format.
        """
        json_tree = "{" + self.print_tree(tree) + "}"
        self.tree_pub.publish(json_tree)


    # =============================================================
    # =======================  PRINT TREE  ========================
    # =============================================================
    def print_tree(self, tree):
        """
            Print an ASCII representation of the bt tree in JSON format
            Its a recursive function!
        """
        json_node = "'" + str("Node") + "':{"
        json_node += "'name':'" + str(tree.name) + "',"
        json_node += "'priority':'" + str(tree.priority) + "',"
        json_node += "'permanence':'" + str(tree.permanence) + "',"
        json_node += "'id':'" + str(tree.id) + "',"
        json_node += "'type':'" + self.get_node_type(tree) + "',"
        json_node += "'status':'" + self.get_node_status(tree) + "',"
        json_node += "'runtime':'" + str(tree.execution_time) + "',"
        json_node += "'children':["

        if tree.children != []:
            for c in tree.children:
                try:
                    json_ch = self.print_tree(c)       # Recursive call
                    json_node += "{" + json_ch + "},"
                except:
                    pass
            json_node = json_node[:-1]  # remove last char ","

        json_node += "]"    # end of children
        json_node += "}"    # end of this node
        return json_node


    # =============================================================
    # =======================  get_node_type  =====================
    # =============================================================
    def get_node_type(self, node):
        if isinstance(node, Selector):
            return "Selector"
        elif isinstance(node, RandomSelector):
            return "RandomSelector"
        elif isinstance(node, Sequence):
            return "Sequence"
        elif isinstance(node, RandomSequence):
            return "RandomSequence"
        elif isinstance(node, Iterator):
            return "Iterator"
        elif isinstance(node, RandomIterator):
            return "RandomIterator"
        elif isinstance(node, Loop):
            return "Loop"
        elif isinstance(node, Invert):
            return "Invert"
        else:
            return "Task"

    # =============================================================
    # =======================  get_node_status  ===================
    # =============================================================
    def get_node_status(self, node):
        if node.status == 0:
            return "Failure"
        elif node.status == 1:
            return "Success"
        elif node.status == 2:
            return "Running"
        else:
            return "Unknown"


# =============================================================
# ==========================  MAIN  ===========================
# =============================================================

if __name__ == '__main__':
    tree = bt_manager()
