// TASK COORDINATOR

// if (mqtt_topic[0] == "interventions")
// NOT INCLUDED IN INTERVENTIONS
if (mqtt_topic[0] == "tunstall") {

    std::string messageValue = new_mqtt2ros_msg->value;

    // Prepare request to the Task_Manager (via service)
    task_manager::addTask task;
    task.request.task_name = "tunstall_manager";
    task.request.task_priority = 5;     //default priority
    task.request.task_permanence = false;
    task.request.task_impact = "reset";
    task.request.task_args.clear();
    
    // params: the code of the trigger itself: eg. BH04_{2022-12-13 15:07:30.553212}
    // this task is transparent to the coordinator/manager, just forward it to tunstall_manager
    task.request.task_args.push_back( messageValue );

    // Keep track of the task to send ACK on finish
    int assigned_task_id = sendTask(task);
    if (assigned_task_id >= 0)
    {
        tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
        os_active = true;
        os_task_id = assigned_task_id;
        if (false) ROS_INFO("[TaskCoordinator] Task has been created..");
    }
    else
    {
        ROS_WARN("[TaskCoordinator] Task has NOT been created.");
        json this_trace;
        this_trace["TaskCoordinator"]="Task cannot be created because of missing component";
        send_ack( "ack", mqtt_topic[0], "none", "none", "none", "Error", this_trace.dump() );
        return;
    }
}

if (mqtt_topic[0] == "interventions")
{
    // ----------------
    //      INFO
    // ----------------
    if (mqtt_topic[2] == "INFO")
    {
        // [...]
        /*
        *  JSON format for Interventions - INFO:
        *  {
        *      “id":"4b3d9598-b320-11e7-abc4-cec278b6b50a"
        *      "userid":“12345”
        *      "ivcode":"INFO"
        *      "time":{"temporality":"timestamp", "t":1494256770.105}
        *
        *      "data":{"code":"WM", "response":"OK/Later/Error"}
        *      "data":{"code":"CT1/CT2", "response":"OK/Later/Error"}
        *      "data":{"code":"CA", "response":"OK/Later/Error"}
        *      "data":{"code":"SQ", "question":""}
        *      "data":{"code":"OS", "objectid":"keys/..."}
        *      "data":{"code":"STOP_OS", "objectid":""}
        *      "data":{"code":"TALK", "text":{"talkcode":"", "description":""} }
        *      "data":{"code":"REM", "remcode":"REM1" }
        *      "data":{"code":"TO", “caller_id":”The name of the Caregiver”, “url”:”teleop_url”, "request_confirmation":true/false}
        *      "data":{"code":"DS", "response":"OK/Later/Error"}
        *      "data":{"code":"MAP"} --> request to get the map of the environment
        *      "data":{"code":"GC", "seqnr": [NUMBER]}
        *   
        *       "data":{"code":"SURVEILLANCE", "state": "on/off"}
        *       "data":{"code":"GET_IMAGE", "command": "get_image"}      	
        *       "data":{"code":"WAIT_SEC", "duration": "20"}
        * 
        *       "data":{"code":"FACE_DETECTION", "task_command": "on"}
                *  }

        /*** TFG02. FACE_DETECTION ***/
        else if (json_msg["data"]["code"] == "FACE_DETECTION")
        {
            ROS_INFO("FACE_DETECTION TASK RECEIVED");

            // GET IMAGE
            task_manager::addTask task;

            // 1. Fire a GET_IMAGE request to the Task_Manager (via service)
            task.request.task_name = "face_detection";
            if (json_msg.find("priority") != json_msg.end())
                task.request.task_priority = json_msg["priority"];
            else
                task.request.task_priority = 5;     //default priority
            task.request.task_permanence = false;
            task.request.task_impact = "none";
            task.request.task_args.clear();
            
            // Argument with the command
            task.request.task_args.push_back(json_msg["data"]["task_command"]);

            // Keep track of the task to send ACK on finish
            int assigned_task_id = sendTask(task);
            if (assigned_task_id >= 0)
            {
                tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                if (false) ROS_INFO("[TaskCoordinator] Task has been created.");
            }
            else
            {
                ROS_WARN("[TaskCoordinator] Task has NOT been created.");
                json this_trace;
                this_trace["TaskCoordinator"]="Task cannot be created because of missing component";
                send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                return;
            }                        
            
        }

# PYTHON

# bt manager
elif task_name == "tunstall_manager":
    # Load additional params of this task
    message = string(task_args[0])

    # Create Task (subtree)
    task = subtree_tunstall_manager(message)
    if task.task_created is True:
        task_tree = task.ROOT
        task_tree.priority = task_prior
        task_tree.permanence = task_perm
        self.insert_new_task_with_priority(task_tree, req.task_impact)


elif task_name == "face_detection":
    # task_args = ["boolean to start/stop"]
    state = task_args[0]

    # Create Task (subtree of Tasks)
    task = subtree_face_detection(state) # Añadir en bt_task_lib

    if task.task_created is True:
        task_tree = task.ROOT
        task_tree.priority = task_prior
        task_tree.permanence = task_perm
        self.insert_new_task_with_priority(task_tree, req.task_impact)


# bt library
class subtree_tunstall_manager(subtree):
    """
    A subtree to implement a simple call to TUNSTALL manager
    """

    def __init__(self, message):
        rospy.loginfo("[bt_manager] New tunstall_manager Task")
        self.pub = rospy.Publisher('/tunstall/trigger', String, queue_size=10)

        self.ROOT = Sequence("TUNSTALL_MANAGER", reset_after=False, announce=False)

        PUBLISH_MESSAGE = GenericTask("PUBLISH_MESSAGE", self.publish_cb, message, reset_after=True)
        self.ROOT.add_child(PUBLISH_MESSAGE)

        self.task_created = True

    def publish_cb(self, msg):
        message = String(msg)
        self.pub.publish(message)
        return TaskStatus.SUCCESS