/** ****************************************************************************************
*  This node implements an API to control and manage the robots of the MAPIR group
*  for the MOVECARE Project.
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
******************************************************************************************** */

#include "task_coordinator/task_coordinator_node.h"
#include "boost/date_time/posix_time/posix_time.hpp"
using namespace std;
using json = nlohmann::json;


// --------------------------------------------
// CTaskCoordinator
//---------------------------------------------
CTaskCoordinator::CTaskCoordinator()
{
    // GENERAL FUNCTIONALITY:
    // ... subscribe to /mqtt2ros topic
    // ... parse json input_messages
    // ... call to TASK_MANAGER service accordingly
    // ... forward the response via MQTT (json)

    ROS_INFO("[TaskCoordinator] Initializing TaskCoordinator node...");

    // Read Local Parameters
    // -----------------------
    ros::NodeHandle pn("~");
    pn.param<bool>("verbose", verbose, false);
    pn.param<bool>("oral_verbose", oral_verbose, false);
    pn.param<std::string>("input_topic",input_topic,"/mqtt2ros");
    pn.param<std::string>("output_topic", output_topic, "/ros2mqtt");
    pn.param<std::string>("task_statistics_file_path", task_statistics_file_path, "task_statistics.txt");
    pn.param<bool>("outdoor_location", outdoor_location, true);
    pn.param<bool>("read_config_from_hgw", use_hgw, false);


    // Load environment and user Info
    // -------------------------------
    dock_station_found = false;
    have_spaces = false;
    bool mandatory_data_found = false;

    // Load locations from json file

    if (verbose) ROS_INFO("\n[TaskCoordinator] LOADING CONFIG FROM  --- FILE ---");

    pn.param<std::string>("local_env_url", local_env_url, "../../maps/mapir_topo_map.json");
    topo_map_json = get_data_from_file(local_env_url, mandatory_data_found);
    if (mandatory_data_found)
    {
        get_env_data_from_json(topo_map_json, mandatory_data_found);
        /*
        if (mandatory_data_found)
        {
            pn.param<std::string>("local_user_url", local_user_url, "../../maps/movecare_user_info.json");
            json json_msg_user =  get_data_from_file(local_user_url, mandatory_data_found);
            if (mandatory_data_found)
                get_user_data_from_json(json_msg_user, mandatory_data_found);
        }
        */
    }    

    if (verbose) ROS_INFO("\n[TaskCoordinator] LOADING CONFIG FROM  --- FILE --- DONE");

    /*
    while (ros::ok() && !mandatory_data_found)
    {
        // 1. Try loading data
        if (use_hgw)
        {
            // Use the Home GateWay server to retrieve user_id, unser_name, user_lanaguage and room information of the environment.
            if (verbose) ROS_INFO("\n[TaskCoordinator] LOADING CONFIG FROM  --- URL ---");

            pn.param<std::string>("hgw_env_url", hgw_env_url, "http://192.168.192.2/api/movecare-floorplan/v1");
            topo_map_json = get_data_from_url(hgw_env_url, mandatory_data_found);
            if (mandatory_data_found)
            {
                get_env_data_from_json(topo_map_json, mandatory_data_found);
                if (mandatory_data_found)
                {
                    pn.param<std::string>("hgw_user_url", hgw_user_url, "http://192.168.192.2/movecare/datamanagement/management/user/[userID]");
                    replace_user_id(hgw_user_url);
                    json json_msg_user = get_data_from_url(hgw_user_url, mandatory_data_found);
                    if (mandatory_data_found)
                        get_user_data_from_json(json_msg_user, mandatory_data_found);
                }
            }
        }
        else
        {
            // There is no server available, load user_id and room locations from local file (JSON format)
            if (verbose) ROS_INFO("\n[TaskCoordinator] LOADING CONFIG FROM  --- FILE ---");

            pn.param<std::string>("local_env_url", local_env_url, "../../maps/movecare_topo_map.json");
            topo_map_json = get_data_from_file(local_env_url, mandatory_data_found);
            if (mandatory_data_found)
            {
                get_env_data_from_json(topo_map_json, mandatory_data_found);
                if (mandatory_data_found)
                {
                    pn.param<std::string>("local_user_url", local_user_url, "../../maps/movecare_user_info.json");
                    json json_msg_user =  get_data_from_file(local_user_url, mandatory_data_found);
                    if (mandatory_data_found)
                        get_user_data_from_json(json_msg_user, mandatory_data_found);
                }
            }
        }

        // 2. Evaluate available data
        if (mandatory_data_found)
        {
            // Set GLOBAL parameters: (to be shared with other nodes -> start-flag)
            // -----------------------
            ros::param::set("/user_id", user_id);
            ros::param::set("/user_name", user_name);
            ros::param::set("/user_surname", user_surname);
            ros::param::set("/user_language", user_language);
            ros::param::set("/topological_map", topo_map_json.dump());
            ros::param::set("/docking_pose", dock_station.poses_str[0]);
        }
        else
        {
            // Likely an internet connection problem.
            // Check if we had the data from a previous execution (re-launched node)
            ros::NodeHandle n;
            if (n.hasParam("/user_id"))
            {
                // Recover previous state
                ros::param::get("/user_id", user_id);
                ros::param::get("/user_name", user_name);
                ros::param::get("/user_surname", user_surname);
                ros::param::get("/user_language", user_language);

                std::string topo_map_json_str;
                ros::param::get("/topological_map", topo_map_json_str);
                topo_map_json = json::parse( topo_map_json_str );
                get_env_data_from_json(topo_map_json, mandatory_data_found);
            }
            else
            {
                // No user info.. keep trying.
                ROS_WARN("[TaskCoordinator] Missing user data (id, name, surname) or Environmental data (docking pose).. I CAN NOT START. WAITING...");
                ros::Duration(5.0).sleep();
            }
        }
    }
    */




    // Wait some time for other nodes to read user data and get ready
    ros::Duration(1.0).sleep();


    // Subscribers & Publishers
    // --------------------------
    mqtt_sub = n.subscribe<diagnostic_msgs::KeyValue>(input_topic,10,&CTaskCoordinator::mqtt2rosCallBack,this);
    task_manager_results_sub = n.subscribe<std_msgs::String>("/bt_manager/task_manager_result",10,&CTaskCoordinator::task_completedCB,this);
    giraff_buttons_sub = n.subscribe<sensor_msgs::Joy>("/giraff_node/buttons",10,&CTaskCoordinator::giraff_buttonsCB,this);

    mqtt_pub = n.advertise<diagnostic_msgs::KeyValue>(output_topic, 1000);
    user_expected_location_pub = n.advertise<std_msgs::String>("/user_expected_location", 1);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "/room_markers", 0 );
    estop_pub = n.advertise<std_msgs::Bool>( "/e_stop", 0 );
    ready_pub = n.advertise<std_msgs::Bool>( "/system_status", 0 );

    // Service Clients
    // ----------------
    task_manager_srv_add = n.serviceClient<task_manager::addTask>("bt_manager/add_new_task");
    if (verbose) ROS_INFO("\n[TaskCoordinator] WAITING FOR NODE bt_manager TO ADVERTISE 'ADD NEW TASK' SRV");
    task_manager_srv_add.waitForExistence();

    task_manager_srv_remove = n.serviceClient<task_manager::removeTask>("bt_manager/remove_task");
    if (verbose) ROS_INFO("\n[TaskCoordinator] WAITING FOR NODE bt_manager TO ADVERTISE 'REMOVE TASK' SRV");
    task_manager_srv_remove.waitForExistence();

    // Initialize variables
    // ---------------------
    ready_to_work = true;                                   // We start in working mode!
    user_location = "";                                     // init user location to empty
    robot_location = "unknown";                             // Robot kidnapped problem!
    alert_active = false;
    teleop_active = false;
    e_stop_active = false;
    offline_warning_active = false;
    last_keepAlive = ros::Time::now();
    tasks_in_execution.clear();                             // Nothing to do at startup!

    //publish initial e-stop status
    std_msgs::Bool estop;
    estop.data = e_stop_active;
    estop_pub.publish(estop);
    std_msgs::Bool ready;
    ready.data = ready_to_work;
    ready_pub.publish(ready);

    // Initialize task statistics
    // --------------------------
    load_task_statistics();
}



CTaskCoordinator::~CTaskCoordinator()
{
    // ensure the robot screen is On
    //interface_srv_call("on", "");
}


// --------------------------
//      REPLACE USER ID
// --------------------------
void CTaskCoordinator::replace_user_id(std::string &s)
{
    // Taking as input a string, replace the "[userID]" substring for the corresponding user_id
    size_t index;
    while ((index = s.find("[userID]")) != string::npos)
        s.replace(index, 8, user_id);
}


// --------------------------------------------------------------
// Send a srv request to the TaskManager node to start a new task
// --------------------------------------------------------------
inline int CTaskCoordinator::sendTask( task_manager::addTask &task )
{
    if(!task_manager_srv_add.call(task))
    {
        ROS_ERROR("[TaskCoordinator] Failed to call service add_new_task with task %s", task.request.task_name.c_str());
        return -1;
    }
    else
    {
        ROS_INFO("[TaskCoordinator] New intervention created with task_id [%i]", task.response.task_id);
        return task.response.task_id;
    }
}


// --------------------------
//      SAY
// --------------------------
void CTaskCoordinator::say(std::string s)
{
    // Prepare request to the Task_Manager (via service) to say something
    task_manager::addTask task;
    task.request.task_name = "say";
    task.request.task_priority = 11;
    task.request.task_permanence = false;
    task.request.task_impact = "none";      // do not reset/cancel other task... keep them in execution
    task.request.task_args.clear();
    task.request.task_args.push_back(s);    // msgs to say

    // Send request to Task Manager
    int assigned_task_id = sendTask(task);
}



//-----------------------------------------------------------------------------------
//                  CALLBACK MQTT --> ROS (new Intervention Request)
//-----------------------------------------------------------------------------------
void CTaskCoordinator::mqtt2rosCallBack(const diagnostic_msgs::KeyValue::ConstPtr& new_mqtt2ros_msg)
{
    // MQTT msg description:
    // new_mqtt2ros_msg->key : contains the MQTT topic where this msg was received
    // new_mqtt2ros_msg->value : contains the JSON message in string format

    /* We may receive different types of data:
     * (a) Intervention requests - interventions/[userID]/(INFO/ALRT/WARN/HELP/ERROR)
     * (b) User location updates - indicators/[userID]/RLC
     * (c) Teleoperation commands (nothing to do.. the Teleop node will handle them directly)
     */

    ROS_DEBUG("[\nTASK COORDINATOR] Received message from topic: %s with value: %s", new_mqtt2ros_msg->key.c_str(), new_mqtt2ros_msg->value.c_str());

    try
    {
        // Parse MQTT topic
        std::vector<std::string> mqtt_topic;
        boost::split(mqtt_topic, new_mqtt2ros_msg->key, boost::is_any_of("/"));

        // just for TUNSTALL triggers
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

        else if (mqtt_topic[0] == "interventions")
        {
            // ----------------
            //      INFO
            // ----------------
            if (mqtt_topic[2] == "INFO")
            {
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
                *  }
                */

                std::string messageValue = new_mqtt2ros_msg->value;
                try{
                    if (verbose) ROS_INFO("\n[TaskCoordinator] ---------------------------");
                    if (verbose) ROS_INFO("[TaskCoordinator] New Intervention received via MQTT: [%s]->%s\n", new_mqtt2ros_msg->key.c_str(),messageValue.c_str());

                    auto json_msg = json::parse( messageValue );

                    if (oral_verbose) say("New Intervention received via MQTT with code " + json_msg["data"]["code"].get<std::string>() );

                    // Should we handle the request?
                    if (!ready_to_work && json_msg["data"]["code"] != "DS" )
                    {
                        // System is OFF, reject intervention and do nothing!
                        json ack_trace;
                        ack_trace["TaskCoordinator"]="System is OFF. Ignoring request";
                        send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                        if (oral_verbose) say("System is off. Ignoring request.");
                        return;
                    }
                    else if (e_stop_active)
                    {
                        // Robot is OFF, reject intervention and do nothing!
                        json ack_trace;
                        ack_trace["TaskCoordinator"]="Robot is in emergency mode (e-stop). Ignoring request";
                        send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                        if (oral_verbose) say("Robot is in emergency mode. Ignoring request");
                        return;
                    }
                    else if (outdoor_location && json_msg["data"]["code"] != "DS" && json_msg["data"]["code"] != "TO")
                    {
                        if (user_location == "OUTDOOR" || user_location == "outdoor" || user_location == "Outdoor"
                            || user_location == "OUTSIDE" || user_location == "outside" || user_location == "Outside")
                        {
                            // User is not at home, reject intervention and do nothing!
                            json ack_trace;
                            ack_trace["TaskCoordinator"]="User location is set OUTDOOR. Ignoring request";
                            send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                            if (oral_verbose) say("User location is set to outdoor. Ignoring request");
                            return;
                        }
                    }

                    // A1. GENERAL INFO INTERVENTIONS
                    //--------------------------------
                    if( json_msg["data"]["code"] == "WM" || json_msg["data"]["code"] == "CT1" || json_msg["data"]["code"] == "CT2"|| json_msg["data"]["code"] == "CA" || json_msg["data"]["code"] == "SQ" )
                    {
                        /* All these interventions require:
                         * 1. Find the User in the Environment and Approach him/her
                         * 2. HRI(code)
                         */

                        // 1. We send all room labels+locations of the environment.
                        // To make the search more efficient, we reorder the rooms according to the information provided by the VC
                        // We set as first-location the one the user is expected to be
                        std::vector<string> location_labels;
                        std::vector<string> location_poses;
                        location_labels.clear();
                        location_poses.clear();
                        for (std::map<string,Tlocation>::iterator it=locations.begin(); it!=locations.end(); it++)
                        {
                            if (it->first == user_location)
                            {
                                // Expected user location, set at first element
                                for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                {
                                    if (false) ROS_INFO("[TaskCoordinator] User expected location is [%s] that corresponds to pose: %s", it->first.c_str(),(*it2).c_str());
                                    location_labels.insert(location_labels.begin(), it->first);
                                    location_poses.insert(location_poses.begin(), *it2);
                                }
                            }
                            else
                            {
                                // Non user location
                                for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                {
                                    location_labels.push_back(it->first);
                                    location_poses.push_back(*it2);
                                }
                            }
                        }


                        //2. Prepare request to the Task_Manager (via service)
                        task_manager::addTask task;
                        task.request.task_name = "movecare_intervention";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "reset";
                        //params: userName(string), HRIcode(int), numRooms(int), roomLabels(string[]), roomPoses(string[]), [Additional Params](string list)
                        task.request.task_args.clear();
                        // Username
                        task.request.task_args.push_back(user_name);
                        // HRI code
                        if( json_msg["data"]["code"] == "WM" )
                            task.request.task_args.push_back("1");                      //intervention body weight remainder
                        else if( json_msg["data"]["code"] == "CT1" )
                            task.request.task_args.push_back("3");                      //intervention cognitive test1
                        else if( json_msg["data"]["code"] == "CT2" )
                            task.request.task_args.push_back("4");                      //intervention cognitive test2
                        else if( json_msg["data"]["code"] == "CA" )
                            task.request.task_args.push_back("6");                      //intervention cognitive activity
                        else if( json_msg["data"]["code"] == "SQ" )
                            task.request.task_args.push_back("5");                      //intervention Spot Question

                        // Num Rooms (locations)
                        task.request.task_args.push_back(std::to_string(location_poses.size()));
                        // Room Labels
                        task.request.task_args.insert(task.request.task_args.end(), location_labels.begin(), location_labels.end());
                        // Room Poses (parsed as strings)
                        task.request.task_args.insert(task.request.task_args.end(), location_poses.begin(), location_poses.end());

                        // Additional parameters?
                        if( json_msg["data"]["code"] == "SQ" )
                        {
                            //intervention Spot Question
                            task.request.task_args.push_back(json_msg["data"]["questioncode"]);
                            task.request.task_args.push_back(json_msg["data"]["answer"]);
                        }
                        else if( json_msg["data"]["code"] == "CT1" )
                        {
                            //intervention Cognitive Test
                            task.request.task_args.push_back(json_msg["userid"]);
                        }
                        else if( json_msg["data"]["code"] == "CT2" )
                        {
                            //intervention Cognitive Test
                            task.request.task_args.push_back(json_msg["userid"]);
                            task.request.task_args.push_back(json_msg["data"]["tests"]);
                        }


                        // Send request to Task Manager and keep track of the task_id to send ACK on completion
                        int assigned_task_id = sendTask(task);
                        if (assigned_task_id >= 0)
                        {
                            tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                            if (false) ROS_INFO("[TaskCoordinator] Task has been scheduled in the TaskManager..");
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


                    // A2. OBJECT SEARCH
                    //-------------------
                    else if( json_msg["data"]["code"] == "OS")
                    {
                        // Prepare request to the Task_Manager (via service)
                        task_manager::addTask task;
                        task.request.task_name = "movecare_rfid_search";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "reset";
                        task.request.task_args.clear();
                        //params (object_id, [room_pose1], [room_pose2], etc...)
                        task.request.task_args.push_back(json_msg["data"]["objectid"]);

                        //Parse all room_labels to Poses and provide them as new params
                        for (json::iterator it_rooms = json_msg["data"]["rooms"].begin(); it_rooms != json_msg["data"]["rooms"].end(); ++it_rooms)
                        {
                            // Get pose of the location where the object is likely to be
                            std::map<string,Tlocation>::iterator it;
                            it = locations.find(*it_rooms);
                            if (it == locations.end())
                                ROS_WARN("[TaskCoordinator] RFID suggested location not found in map! Skipping suggestion.");
                            else
                            {
                                // consider all the poses associated to this Space
                                for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                    task.request.task_args.push_back(*it2);   //object likely location (pose parsed as string)
                            }
                        }

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
                            send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                            return;
                        }
                    }


                    // A3. STOP_OS
                    //--------------
                    else if (json_msg["data"]["code"]=="STOP_OS")
                    {
                        // Kill current ObjectSearch task
                        if (os_active)
                        {
                            task_manager::removeTask task_r;
                            task_r.request.task_id = os_task_id;
                            task_r.request.info = "OS cancelled by request STOP_OS";
                            if(!task_manager_srv_remove.call(task_r))
                                ROS_ERROR("[TaskCoordinator] Failed to call service remove_task with task_id %i", task_r.request.task_id);
                        }
                    }


                    // A4. TALK // REMINDER
                    //-----------------------
                    else if( json_msg["data"]["code"] == "TALK" || json_msg["data"]["code"] == "REM")
                    {
                        /* There are two versions of these interventions
                         * 1. Locate the user and then deliver the message
                         * 2. Just talk in place, NO navigation
                         */

                        if( json_msg["data"]["find_user"] == true )
                        {
                            // 1. We send all room labels+locations of the environment.
                            // To make the search more efficient, we reorder the rooms according to the information provided by the VC
                            // We set as first-location the one the user is expected to be
                            std::vector<string> location_labels;
                            std::vector<string> location_poses;
                            location_labels.clear();
                            location_poses.clear();
                            for (std::map<string,Tlocation>::iterator it=locations.begin(); it!=locations.end(); it++)
                            {
                                if (it->first == user_location)
                                {
                                    // Expected user location, set at first element
                                    for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                    {
                                        if (false) ROS_INFO("[TaskCoordinator] User expected location is [%s] that corresponds to pose: %s", it->first.c_str(),(*it2).c_str());
                                        location_labels.insert(location_labels.begin(), it->first);
                                        location_poses.insert(location_poses.begin(), *it2);
                                    }
                                }
                                else
                                {
                                    // Non user location
                                    for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                    {
                                        location_labels.push_back(it->first);
                                        location_poses.push_back(*it2);
                                    }
                                }
                            }

                            //2. Fire request to the Task_Manager (via service)
                            task_manager::addTask task;
                            task.request.task_name = "movecare_intervention";
                            if (json_msg.find("priority") != json_msg.end())
                                task.request.task_priority = json_msg["priority"];
                            else
                                task.request.task_priority = 5;     //default priority
                            task.request.task_permanence = false;
                            task.request.task_impact = "reset";
                            //params: userName(string), HRIcode(int), numRooms(int), roomLabels(string[]), roomPoses(string[]), [texts_to_talk](string list)
                            task.request.task_args.clear();
                            task.request.task_args.push_back(user_name);                     //user name
                            if (json_msg["data"]["code"] == "TALK")
                                task.request.task_args.push_back("9");                          //hri_code = talk (9)
                            else
                                task.request.task_args.push_back("7");                          //hri_code = reminder (7)
                            task.request.task_args.push_back(std::to_string(location_poses.size()));                                             // Num poses (locations)
                            task.request.task_args.insert(task.request.task_args.end(), location_labels.begin(), location_labels.end());    // Room Labels
                            task.request.task_args.insert(task.request.task_args.end(), location_poses.begin(), location_poses.end());      // Room Poses (parsed as strings)
                            if (json_msg["data"]["code"] == "TALK")
                                task.request.task_args.push_back(json_msg["data"]["text"]["talkcode"]); // msgs to talk
                            else
                                task.request.task_args.push_back(json_msg["data"]["remcode"]);          // reminder code <string>


                            // Keep track of the task to send ACK on finish
                            int assigned_task_id = sendTask(task);
                            if (assigned_task_id >= 0)
                            {
                                tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                                if (false) ROS_INFO("[TaskCoordinator] Task has been created..");
                            }
                            else
                            {
                                ROS_WARN("[TaskCoordinator] Task hsa NOT been created..");
                                json this_trace;
                                this_trace["TaskCoordinator"]="Task cannot be created because of missing component";
                                send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                                return;
                            }
                        }
                        else
                        {
                            // JUST TALK/REM (No navigation involved)
                            task_manager::addTask task;
                            if (json_msg["data"]["code"] == "TALK")
                            {
                                // TALK
                                //1. Fire a TALK request to the Task_Manager (via service)
                                task.request.task_name = "say";
                                if (json_msg.find("priority") != json_msg.end())
                                    task.request.task_priority = json_msg["priority"];
                                else
                                    task.request.task_priority = 5;     //default priority
                                task.request.task_permanence = false;
                                task.request.task_impact = "none";
                                task.request.task_args.clear();
                                //params (text)
                                task.request.task_args.push_back(json_msg["data"]["text"]["talkcode"]); // msgs to talk
                            }
                            else
                            {
                                // REM
                                //1. Fire request to the Task_Manager (via service)
                                task.request.task_name = "rem";
                                if (json_msg.find("priority") != json_msg.end())
                                    task.request.task_priority = json_msg["priority"];
                                else
                                    task.request.task_priority = 5;     //default priority
                                task.request.task_permanence = false;
                                task.request.task_impact = "none";
                                task.request.task_args.clear();
                                //params (hri_code, username, remcode)
                                task.request.task_args.push_back("7");                           //hri_code = reminder (7)
                                task.request.task_args.push_back(user_name);                     //user name
                                task.request.task_args.push_back(json_msg["data"]["remcode"]);
                            }

                            // Keep track of the task to send ACK on finish
                            int assigned_task_id = sendTask(task);
                            if (assigned_task_id >= 0)
                            {
                                tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                                if (false) ROS_INFO("[TaskCoordinator] Task has been created..");
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
                    }


                    // A5. COME HERE (GC)
                    //-----------------------
                    else if( json_msg["data"]["code"] == "GC" )
                    {
                         // Locate the user and then deliver the message


                        // 1. We send all room labels+locations of the environment.
                        // To make the search more efficient, we reorder the rooms according to the information provided by the VC
                        // We set as first-location the one the user is expected to be
                        std::vector<string> location_labels;
                        std::vector<string> location_poses;
                        location_labels.clear();
                        location_poses.clear();
                        for (std::map<string,Tlocation>::iterator it=locations.begin(); it!=locations.end(); it++)
                        {
                            if (it->first == user_location)
                            {
                                // Expected user location, set at first element
                                for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                {
                                    if (false) ROS_INFO("[TaskCoordinator] User expected location is [%s] that corresponds to pose: %s", it->first.c_str(),(*it2).c_str());
                                    location_labels.insert(location_labels.begin(), it->first);
                                    location_poses.insert(location_poses.begin(), *it2);
                                }
                            }
                            else
                            {
                                // Non user location
                                for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                {
                                    location_labels.push_back(it->first);
                                    location_poses.push_back(*it2);
                                }
                            }
                        }

                        //2. Fire request to the Task_Manager (via service)
                        task_manager::addTask task;
                        task.request.task_name = "movecare_intervention";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "reset";
                        //params: userName(string), HRIcode(int), numRooms(int), roomLabels(string[]), roomPoses(string[]), [texts_to_talk](string list)
                        task.request.task_args.clear();
                        task.request.task_args.push_back(user_name);                     //user name
                        task.request.task_args.push_back("7");                           //hri_code = reminder (7)
                        task.request.task_args.push_back(std::to_string(location_poses.size()));                                        // Num poses (locations)
                        task.request.task_args.insert(task.request.task_args.end(), location_labels.begin(), location_labels.end());    // Room Labels
                        task.request.task_args.insert(task.request.task_args.end(), location_poses.begin(), location_poses.end());      // Room Poses (parsed as strings)
                        task.request.task_args.push_back("REM18");          // reminder code <string>


                        // Keep track of the task to send ACK on finish
                        int assigned_task_id = sendTask(task);
                        if (assigned_task_id >= 0)
                        {
                            tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                            if (false) ROS_INFO("[TaskCoordinator] Task has been created..");
                        }
                        else
                        {
                            ROS_WARN("[TaskCoordinator] Task hsa NOT been created..");
                            json this_trace;
                            this_trace["TaskCoordinator"]="Task cannot be created because of missing component";
                            send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                            return;
                        }
                    }


                    // A5. DOCK
                    //-----------
                    else if( json_msg["data"]["code"] == "DS")
                    {
                        // Get pose of the Docking station or quit if not found
                        if (!dock_station_found)
                        {
                           ROS_WARN("[TaskCoordinator] Docking_Station location not found in the map!. Skipping request.");
                           json this_trace;
                           this_trace["TaskCoordinator"]="Docking Station Location Not Found";
                           send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                           return;
                        }

                        // Fire request to the Task_Manager (via service)
                        task_manager::addTask task;
                        task.request.task_name = "dock";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "reset";
                        task.request.task_args.clear();
                        //params [docking_pose, speech]
                        task.request.task_args.push_back(dock_station.poses_str[0]);    // dock station pose
                        task.request.task_args.push_back("false");                      // speech

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



                    // A6. TELEOPERATION
                    //-------------------
                    else if( json_msg["data"]["code"] == "TO")
                    {
                        if (!teleop_active)
                        {
                            /*
                            *  {
                            *      “id":"taskID"
                            *      "userid":“xxx”
                            *      "ivcode":"INFO"
                            *      "time":{"temporality":"timestamp", "t":1494256770.105>}
                            *      "data":{"code":"TO", “caller_id":”The name of the Caregiver”, “url”:”teleop_url”, "request_confirmation":true/false}
                            *  }
                            */


                            // 0. We need to look for the user, so we send all room labels+locations of the environment.
                            // To make the search more efficient, we reorder the rooms according to the information provided by the VC
                            // We set as first-location the one the user is expected to be
                            std::vector<string> location_labels;
                            std::vector<string> location_poses;
                            location_labels.clear();
                            location_poses.clear();
                            for (std::map<string,Tlocation>::iterator it=locations.begin(); it!=locations.end(); it++)
                            {
                                if (it->first == user_location)
                                {
                                    // Expected user location, set at first element
                                    for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                    {
                                        if (false) ROS_INFO("[TaskCoordinator] User expected location is [%s] that corresponds to pose: %s", it->first.c_str(),(*it2).c_str());
                                        location_labels.insert(location_labels.begin(), it->first);
                                        location_poses.insert(location_poses.begin(), *it2);
                                    }
                                }
                                else
                                {
                                    // Non user location
                                    for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                    {
                                        location_labels.push_back(it->first);
                                        location_poses.push_back(*it2);
                                    }
                                }
                            }


                            // 1. Prepare a Teleoperation request to the TaskManager
                            task_manager::addTask task;
                            task.request.task_name = "teleoperation";
                            if (json_msg.find("priority") != json_msg.end())
                                task.request.task_priority = json_msg["priority"];
                            else
                                task.request.task_priority = 9;     //default priority is high!
                            task.request.task_permanence = false;
                            task.request.task_impact = "reset";
                            task.request.task_args.clear();                            
                            // parameters: callerID, callerName, confURL, teleopURL, requestConfirmation, numLocation, locationLabels, locationPoses):
                            task.request.task_args.push_back(json_msg["data"]["caller_id"]);
                            task.request.task_args.push_back(json_msg["data"]["caller_name"]);
                            task.request.task_args.push_back(json_msg["data"]["confirmation_url"]);
                            task.request.task_args.push_back(json_msg["data"]["teleop_url"]);
                            if (json_msg["data"].find("request_confirmation") != json_msg["data"].end() ){
                                if (json_msg["data"]["request_confirmation"] == true)
                                    task.request.task_args.push_back("true");
                                else
                                    task.request.task_args.push_back("false");
                            }
                            else
                                task.request.task_args.push_back("true");

                            // Num Rooms (locations)
                            task.request.task_args.push_back(std::to_string(location_poses.size()));
                            // Room Labels
                            task.request.task_args.insert(task.request.task_args.end(), location_labels.begin(), location_labels.end());
                            // Room Poses (parsed as strings)
                            task.request.task_args.insert(task.request.task_args.end(), location_poses.begin(), location_poses.end());


                            // Request to TaskManager
                            int assigned_task_id = sendTask(task);
                            if (assigned_task_id >= 0)
                            {
                                teleop_active = true;
                                tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                                if (false) ROS_INFO("[TaskCoordinator] Task has been created..");
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
                        else
                            if (verbose) ROS_INFO("[TaskCoordinator] A Teleoperation Task already in action.");
                    }


                    // A7. GO TO POINT (for navigation testing)
                    //-------------------------------------------
                    else if( json_msg["data"]["code"] == "GO" )
                    {
                        //1. Prepare request to the Task_Manager (via service)
                        task_manager::addTask task;
                        task.request.task_name = "go_to_point";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "reset";
                        //params: label(string), roomPose(string[])
                        task.request.task_args.clear();

                        // Get pose of the location to go
                        std::map<string,Tlocation>::iterator it;
                        it = locations.find(json_msg["data"]["label"]);
                        if (it == locations.end())
                        {
                            ROS_WARN("[TaskCoordinator] GO_TO label not found in map! Skipping request.");
                            return;
                        }
                        else
                        {
                            task.request.task_args.push_back( json_msg["data"]["label"] );

                            // Select the pose associated to this Space (may have multiple poses)
                            if (it->second.poses_str.size() > json_msg["data"]["pose_idx"])
                            {
                                task.request.task_args.push_back( it->second.poses_str[json_msg["data"]["pose_idx"]] );
                            }
                            else
                            {
                                ROS_WARN("[TaskCoordinator] GO_TO pose_idx not found in map! Skipping request.");
                                return;
                            }
                        }

                        // Send request to Task Manager and keep track of the task_id to send ACK on completion
                        int assigned_task_id = sendTask(task);
                        if (assigned_task_id >= 0)
                        {
                            tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                            if (false) ROS_INFO("[TaskCoordinator] Task has been scheduled in the TaskManager..");
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

                    // A8. GET MAP through MQTT (base64 encoded)
                    //-------------------------------------------
                    else if( json_msg["data"]["code"] == "MAP" )
                    {
                        // This is a special case. The only task is to send a text_file (encoded in base64) through the ACK msg
                        std::ifstream map_encoded64;
                        json this_trace;
                        std::string map_encoded64_str;
                        map_encoded64.open(ros::package::getPath("missions_pkg")+"/maps/movecare_map.base64");
                        if (map_encoded64.is_open())
                        {
                            // single line file
                            std::stringstream strStream;
                            strStream << map_encoded64.rdbuf();  //read the file
                            map_encoded64_str = strStream.str(); //str holds the content of the file
                            //getline(map_encoded64,map_encoded64_str);
                            map_encoded64.close();
                            this_trace["TaskCoordinator"] = "Sending Map encoded in base64";
                        }
                        else
                            this_trace["TaskCoordinator"] = "ERROR Map encoded in base64 not found";

                        // Send ACK
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], map_encoded64_str, this_trace.dump() );
                    }

                    /*** TFG01. GET IMAGE AND SEND IT VIA MQTT ***/
                    else if (json_msg["data"]["code"] == "GET_IMAGE")
                    {
                        ROS_INFO("GET_IMAGE TASK RECEIVED");

                        // GET IMAGE
                        task_manager::addTask task;

                        // 1. Fire a GET_IMAGE request to the Task_Manager (via service)
                        task.request.task_name = "get_image";
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
                    
                    /*** TFG02. ACTIVATE SURVEILLANCE MODE ***/
                    else if (json_msg["data"]["code"] == "SURVEILLANCE")
                    {
                        task_manager::addTask task;

                        ROS_INFO("SURVEILLANCE TASK RECEIVED");
                        
                        // SURVEILLANCE
                        //1. Fire a SURVEILLANCE request to the Task_Manager (via service)
                        task.request.task_name = "surveillance";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "none";
                        task.request.task_args.clear();
                            
                        // Argument to check if the mode has to start or stop
                        task.request.task_args.push_back(json_msg["data"]["state"]); // state required

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

                    /*** TFG02. FACE_DETECTION ***/
                    else if (json_msg["data"]["code"] == "FACE_DETECTION")
                    {
                        ROS_INFO("FACE_DETECTION TASK RECEIVED");

                        // GET IMAGE
                        task_manager::addTask task;

                        // 1. Fire a FACE_DETECTION request to the Task_Manager (via service)
                        task.request.task_name = "face_detection_tfg";
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

                    else if (json_msg["data"]["code"] == "WAIT_SEC")
                    {
                        ROS_INFO("WAIT_SEC TASK RECEIVED");

                        // GET IMAGE
                        task_manager::addTask task;

                        // 1. Fire a WAIT_SEC request to the Task_Manager (via service)
                        task.request.task_name = "wait";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "none";
                        task.request.task_args.clear();
                        
                        // Argument with the duration in seconds
                        task.request.task_args.push_back(json_msg["data"]["duration"]);

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
                    /*** Unknown INFO Intervention ***/
                    else
                    {
                        ROS_ERROR("[TaskCoordinator] Intervention not implemented yet! ... doing nothing!");
                        json this_trace;
                        this_trace["TaskCoordinator"] = "INFO Intervention Code not Implemented";
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                        return;
                    }


                }catch (exception e)
                {
                    ROS_ERROR("[taskCoordinator] Malformed JSON request in new intervention [%s]", e.what());
                    if (oral_verbose) say("Malformed JSON data. Ignoring request");
                    return;
                }
            }//end interventions-INFO



            // ----------------
            //      HELP
            // ----------------
            else if (mqtt_topic[2] == "HELP")
            {
                /* HELP
                 *
                 * JSON format for Help scenario:
                 *  {
                 *      "id":<UUID>
                 *      "userid":<userid>
                 *      "ivcode":"HELP"
                 *      "time":{"temporality":"timestamp", "t":<timestamp>}
                 *      "data":{"code":"CH","seqnr":[1-3]}
                 *  }
                 */

                std::string messageValue = new_mqtt2ros_msg->value;
                try{
                    auto json_msg = json::parse( messageValue );
                    if (oral_verbose) say("New Help request received via MQTT with code " + json_msg["data"]["code"].get<std::string>() );

                    // Handle HELP request
                    if( json_msg["data"]["code"] == "CH" )
                    {
                        if (verbose) ROS_WARN("\n[TaskCoordinator] ---------------------------------");
                        if (verbose) ROS_WARN("[TaskCoordinator] New CALL FOR HELP received via MQTT: [%s]->%s\n", new_mqtt2ros_msg->key.c_str(),messageValue.c_str());

                        // Only react if the seqnr <= 3 (three first attemps for help)
                        std::string iqnr_str = json_msg["data"]["seqnr"];
                        int sqnr = atoi(iqnr_str.c_str());

                        if (sqnr<= 3 )
                        {
                            // Should we handle the request?
                            if (!ready_to_work)
                            {
                                // System is OFF, reject intervention and do nothing!
                                json ack_trace;
                                ack_trace["TaskCoordinator"]="System is OFF. Ignoring request";
                                send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                                if (oral_verbose) say("System is OFF. Ignoring request. ");
                                return;
                            }
                            else if (e_stop_active)
                            {
                                // Robot is OFF, reject intervention and do nothing!
                                json ack_trace;
                                ack_trace["TaskCoordinator"]="Robot is in emergency mode (e-stop). Ignoring request";
                                send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                                if (oral_verbose) say("Robot is in emergency mode. Ignoring request");
                                return;
                            }
                            /*else if (outdoor_location)
                            {
                                if (user_location == "OUTDOOR" || user_location == "outdoor" || user_location == "Outdoor"
                                    || user_location == "OUTSIDE" || user_location == "outside" || user_location == "Outside")
                                {
                                    // User is not at home, reject intervention and do nothing!
                                    json ack_trace;
                                    ack_trace["TaskCoordinator"]="User location is set OUTDOOR. Ignoring request";
                                    send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                                    if (oral_verbose) say("User location is set to outdoor. Ignoring request");
                                    return;
                                }
                            }*/

                            if (verbose) ROS_WARN("[TaskCoordinator] Activating CallForHelp with seqnr=%i", sqnr );

                            // Multiple calls to CH are prone to happen (ignore if alaready in HELP mode)
                            if (alert_active)
                            {
                                ROS_WARN("[TaskCoordinator] HELP scenario already in action.... ignoring this new request");
                                json this_trace;
                                this_trace["TaskCoordinator"] = "HELP already in action. Ignoring multiple calls";
                                send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Yes", this_trace.dump() );
                                return;
                            }


                            /* This intervention requires:
                             * 1. Find the User in the Environment
                             * 2. User Approach
                             * 3. HRI(code=2) Call For Help
                             */

                            // 1. We send all room labels+locations of the environment.
                            // To make the search more efficient, we reorder the rooms according to the information provided by the VC
                            // We set as first-location the one the user is expected to be
                            std::vector<string> location_labels;
                            std::vector<string> location_poses;
                            location_labels.clear();
                            location_poses.clear();
                            for (std::map<string,Tlocation>::iterator it=locations.begin(); it!=locations.end(); it++)
                            {
                                if (it->first == user_location)
                                {
                                    // Expected user location, set at first element
                                    for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                    {
                                        if (false) ROS_INFO("[TaskCoordinator] User expected location is [%s] that corresponds to pose: %s", it->first.c_str(),(*it2).c_str());
                                        location_labels.insert(location_labels.begin(), it->first);
                                        location_poses.insert(location_poses.begin(), *it2);
                                    }
                                }
                                else
                                {
                                    // Non user location
                                    for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                    {
                                        location_labels.push_back(it->first);
                                        location_poses.push_back(*it2);
                                    }
                                }
                            }


                            // Fire request to the Task_Manager (via service)
                            task_manager::addTask task;
                            task.request.task_name = "movecare_intervention";
                            if (json_msg.find("priority") != json_msg.end())
                                task.request.task_priority = json_msg["priority"];
                            else
                                task.request.task_priority = 9;     //default priority
                            task.request.task_permanence = false;
                            task.request.task_impact = "cancel";                            // Cancel all other tasks with lower priority!
                            //params: userName(string), HRIcode(int), numRooms(int), roomLabels(string[]), roomPoses(string[]), [texts_to_talk](string list)
                            task.request.task_args.clear();
                            task.request.task_args.push_back(user_name);                     //user name
                            task.request.task_args.push_back("2");                          //intervention Call For Help
                            task.request.task_args.push_back(std::to_string(location_poses.size()));                                             // Num Rooms (locations)
                            task.request.task_args.insert(task.request.task_args.end(), location_labels.begin(), location_labels.end());    // Room Labels
                            task.request.task_args.insert(task.request.task_args.end(), location_poses.begin(), location_poses.end());      // Room Poses (parsed as strings)

                            // Send srv request!
                            int assigned_task_id = sendTask(task);
                            if (assigned_task_id >= 0)
                            {
                                tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                                alert_active = true;
                                if (false) ROS_INFO("[TaskCoordinator] Task has been created..");

                                // Send ACK to inform the system the robot is on its way!
                                json this_trace;
                                this_trace["TaskCoordinator"] = "Call For Help requested...starting CH scenario.";
                                send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Yes", this_trace.dump() );
                            }
                            else
                            {
                                ROS_WARN("[TaskCoordinator] Task has NOT been created.");
                                alert_active = false;
                                json this_trace;
                                this_trace["TaskCoordinator"]="Task cannot be created because of missing component";
                                send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                                return;
                            }
                        }
                        else
                        {
                            // Call for Help request is >3 attemp... Robot will do nothing since is the HGW responsible for handling the situation
                            ROS_WARN("[TaskCoordinator] CallForHelp ignored since seqnr=%i (>3 is HGW responsability)",sqnr );
                            if (oral_verbose) say("CallForHelp request ignored with sequence number higher than three.");
                        }


                    }
                    /*** Unknown HELP REQUEST***/
                    else
                    {
                        ROS_ERROR("[TaskCoordinator] HELP code not implemented yet! ... doing nothing!");
                        json this_trace;
                        this_trace["TaskCoordinator"]="HELP Code not Implemented";
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                        return;
                    }

                }catch (exception e)
                {
                    ROS_ERROR("[taskCoordinator] Malformed JSON request for intervention Alert... ignoring it");
                    if (oral_verbose) say("Malformed JSON data. Ignoring Help request.");
                    return;
                }
            }//end intervention-HELP



            // ----------------
            //      ALERT
            // ----------------
            else if (mqtt_topic[2] == "ALRT")
            {
                /*
                 * JSON format for Alerts:
                 *  {
                 *      "id":<UUID>
                 *      "userid":<userid>
                 *      "ivcode":"ALRT"
                 *      "time":{"temporaliy":"timestamp", "t":<timestamp>}
                 *      "data":{"code":"LW/GW", "response":"OK/Error", ...}
                 *      "data":{"code":"OFF/ON"}
                 *  }
                 */

                std::string messageValue = new_mqtt2ros_msg->value;
                try{
                    if (verbose) ROS_WARN("\n[TaskCoordinator] --------------------------------- ");
                    if (verbose) ROS_WARN("[TaskCoordinator] New ALERT received via MQTT: [%s]->%s\n", new_mqtt2ros_msg->key.c_str(),messageValue.c_str());
                    auto json_msg = json::parse( messageValue );

                    if (oral_verbose) say("New Alert received via MQTT with code " + json_msg["data"]["code"].get<std::string>() );

                    // 1. We always handle ALRT request code=ON/OFF, since are the ones used to turn system ON/OFF
                    // SMART-BUTTON-ON
                    if( json_msg["data"]["code"] == "ON")
                    {
                        // Smart button released!. Back to bussiness!!
                        ready_to_work = true;
                        // Share status
                        std_msgs::Bool ready;
                        ready.data = ready_to_work;
                        ready_pub.publish(ready);
                        // Send ACK
                        json this_trace;
                        this_trace["TaskCoordinator"]="Smart button ON. Robot is ready to work.";
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "OK", this_trace.dump() );
                        if (verbose) ROS_WARN("\n[TaskCoordinator] SMART BUTTON ON --> READY TO WORK");
                        return;
                    }
                    // SMART-BUTTON-off
                    else if( json_msg["data"]["code"] == "OFF")
                    {
                        // Smart button pressed!. Stop everything and return to Docking
                        //0. Fire request to cancel all the Tasks to Task_Manager (via service)
                        task_manager::removeTask task_r;
                        task_r.request.task_id = -1;
                        task_r.request.info = "Task cancelled by SMART-BUTTON-OFF";
                        if(!task_manager_srv_remove.call(task_r))
                            ROS_ERROR("[TaskCoordinator] Failed to call service remove_task with task_id %i", task_r.request.task_id);


                        // 1. Block any other intervention until notified
                        ready_to_work = false;
                        // Share status
                        std_msgs::Bool ready;
                        ready.data = ready_to_work;
                        ready_pub.publish(ready);


                        // 2. Command return to docking
                        // Get pose of the Docking station or quit if not found
                        if (!dock_station_found)
                        {
                           ROS_WARN("[TaskCoordinator] Docking_Station location not found in the map!. Skipping request.");
                           json this_trace;
                           this_trace["TaskCoordinator"]="Docking Station Location Not Found";
                           send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                           return;
                        }

                        // Fire request to the Task_Manager (via service)
                        task_manager::addTask task;
                        task.request.task_name = "dock";
                        task.request.task_priority = 8;                      // Highest priority task!
                        task.request.task_permanence = false;
                        task.request.task_impact = "cancel";                 // Cancel all pending tasks!
                        task.request.task_args.clear();
                        //params [docking_pose, speech]
                        task.request.task_args.push_back(dock_station.poses_str[0]);  // dock station pose
                        task.request.task_args.push_back("false");          // speech

                        // Execute task (do not keep track, as we are sending ACK now)
                        int assigned_task_id = sendTask(task);
                        if (assigned_task_id < 0)
                        {
                            ROS_WARN("[TaskCoordinator] Dock action has NOT been created.");
                            json this_trace;
                            this_trace["TaskCoordinator"]="Task DOcking cannot be created because of missing component";
                            send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                            return;
                        }

                        // Send ACK on OFF request
                        json this_trace;
                        this_trace["TaskCoordinator"]="Smart button OFF. Robot will remain in the Docking Station.";
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "OK", this_trace.dump() );
                        if (verbose) ROS_WARN("\n[TaskCoordinator] SMART BUTTON ON --> READY TO WORK");
                        return;
                    }


                    //2. Process other requests only if system is ON
                    if (!ready_to_work)
                    {
                        // System is OFF, reject intervention and do nothing!
                        json ack_trace;
                        ack_trace["TaskCoordinator"]="System is OFF. Ignoring request";
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                        return;
                    }
                    else if (e_stop_active)
                    {
                        // Robot is OFF, reject intervention and do nothing!
                        json ack_trace;
                        ack_trace["TaskCoordinator"]="Robot is in emergency mode (e-stop). Ignoring request";
                        send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                        if (oral_verbose) say("Robot is in emergency mode. Ignoring request");
                        return;
                    }
                    else if (outdoor_location)
                    {
                        if (user_location == "OUTDOOR" || user_location == "outdoor" || user_location == "Outdoor"
                            || user_location == "OUTSIDE" || user_location == "outside" || user_location == "Outside")
                        {
                            // User is not at home, reject intervention and do nothing!
                            json ack_trace;
                            ack_trace["TaskCoordinator"]="User location is set OUTDOOR. Ignoring request";
                            send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                            if (oral_verbose) say("User location is set to outdoor. Ignoring request");
                            return;
                        }
                    }


                    // Robot is fine --> process request
                    if( json_msg["data"]["code"] == "LW" ||  json_msg["data"]["code"] == "GW" )
                    {
                        ROS_WARN("[TaskCoordinator] Important Weight change on user detected ... NOTHING IMPLEMENTED ON THE ROBOT!");
                    }
                    /*** Unknown Alert***/
                    else
                    {
                        ROS_ERROR("[TaskCoordinator] Alert code not implemented yet! ... doing nothing!");
                        json this_trace;
                        this_trace["TaskCoordinator"]="Alert Code not Implemented";
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", this_trace.dump() );
                        return;
                    }
                }catch (exception e)
                {
                    ROS_ERROR("[taskCoordinator] Malformed JSON request for intervention Alert... ignoring it");
                    if (oral_verbose) say("Malformed JSON data. Ignoring Alert request.");
                    return;
                }
            }//end intervention-ALRT



            // ----------------
            //      WARNING
            // ----------------
            else if (mqtt_topic[2] == "WARN")
            {
                /*
                *  JSON format for Interventions:
                *  {
                *      “id":"4b3d9598-b320-11e7-abc4-cec278b6b50a"
                *      "userid":“12345”
                *      "ivcode":"INFO"
                *      "time":{"temporality":"timestamp", "t":1494256770.105>}
                *
                *      "data":{"code":"VSC", "response":"OK/Later/Error"}
                *  }
                */
                std::string messageValue = new_mqtt2ros_msg->value;
                try
                {
                    if (verbose) ROS_WARN("\n[TaskCoordinator] --------------------------------- ");
                    if (verbose) ROS_WARN("[TaskCoordinator] New WARN received via MQTT: [%s]->%s\n", new_mqtt2ros_msg->key.c_str(),messageValue.c_str());
                    auto json_msg = json::parse( messageValue );

                    if (oral_verbose) say("New Warning intervention received via MQTT with code " + json_msg["data"]["code"].get<std::string>() );

                    // Should we handle the request?
                    if (!ready_to_work)
                    {
                        // System is OFF, reject intervention and do nothing!
                        json ack_trace;
                        ack_trace["TaskCoordinator"]="System is OFF. Ignoring request";
                        send_ack( "ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                        if (oral_verbose) say("System is off. Ignoring request.");
                        return;
                    }
                    else if (e_stop_active)
                    {
                        // Robot is OFF, reject intervention and do nothing!
                        json ack_trace;
                        ack_trace["TaskCoordinator"]="Robot is in emergency mode (e-stop). Ignoring request";
                        send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                        if (oral_verbose) say("Robot is in emergency mode. Ignoring request");
                        return;
                    }
                    else if (outdoor_location)
                    {
                        if (user_location == "OUTDOOR" || user_location == "outdoor" || user_location == "Outdoor"
                            || user_location == "OUTSIDE" || user_location == "outside" || user_location == "Outside")
                        {
                            // User is not at home, reject intervention and do nothing!
                            json ack_trace;
                            ack_trace["TaskCoordinator"]="User location is set OUTDOOR. Ignoring request";
                            send_ack("ack", mqtt_topic[2], json_msg["id"], json_msg["userid"], json_msg["data"]["code"], "Error", ack_trace.dump());
                            if (oral_verbose) say("User location is set to outdoor. Ignoring request");
                            return;
                        }
                    }


                    // Unreliable WM - Repeat
                    //------------------------
                    if( json_msg["data"]["code"] == "VSC" )
                    {
                        // 1. We send all room labels+locations of the environment.
                        // To make the search more efficient, we reorder the rooms according to the information provided by the VC
                        // We set as first-location the one the user is expected to be
                        std::vector<string> location_labels;
                        std::vector<string> location_poses;
                        location_labels.clear();
                        location_poses.clear();
                        for (std::map<string,Tlocation>::iterator it=locations.begin(); it!=locations.end(); it++)
                        {
                            if (it->first == user_location)
                            {
                                // Expected user location, set at first element
                                for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                {
                                    if (false) ROS_INFO("[TaskCoordinator] User expected location is [%s] that corresponds to pose: %s", it->first.c_str(),(*it2).c_str());
                                    location_labels.insert(location_labels.begin(), it->first);
                                    location_poses.insert(location_poses.begin(), *it2);
                                }
                            }
                            else
                            {
                                // Non user location
                                for (std::vector<string>::iterator it2=it->second.poses_str.begin(); it2!=it->second.poses_str.end(); it2++)
                                {
                                    location_labels.push_back(it->first);
                                    location_poses.push_back(*it2);
                                }
                            }
                        }


                        //2. Prepare request to the Task_Manager (via service)
                        task_manager::addTask task;
                        task.request.task_name = "movecare_intervention";
                        if (json_msg.find("priority") != json_msg.end())
                            task.request.task_priority = json_msg["priority"];
                        else
                            task.request.task_priority = 5;     //default priority
                        task.request.task_permanence = false;
                        task.request.task_impact = "reset";
                        //params: userName(string), HRIcode(int), numRooms(int), roomLabels(string[]), roomPoses(string[]), [Additional Params](string list)
                        task.request.task_args.clear();
                        // Username
                        task.request.task_args.push_back(user_name);
                        // HRI code
                        task.request.task_args.push_back("6");                      //intervention body weight repetition

                        // Num Rooms (locations)
                        task.request.task_args.push_back(std::to_string(location_poses.size()));
                        // Room Labels
                        task.request.task_args.insert(task.request.task_args.end(), location_labels.begin(), location_labels.end());
                        // Room Poses (parsed as strings)
                        task.request.task_args.insert(task.request.task_args.end(), location_poses.begin(), location_poses.end());


                        // Send request to Task Manager and keep track of the task_id to send ACK on completion
                        int assigned_task_id = sendTask(task);
                        if (assigned_task_id >= 0)
                        {
                            tasks_in_execution.insert( std::pair<int,string>(assigned_task_id, messageValue) );
                            if (false) ROS_INFO("[TaskCoordinator] Task has been scheduled in the TaskManager..");
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
                }catch (exception e)
                {
                    ROS_ERROR("[taskCoordinator] Malformed JSON request for intervention Warning... ignoring it");
                    if (oral_verbose) say("Malformed JSON data. Ignoring Warning request.");
                    return;
                }
            }//end intervention-WARN


        }// end INTERVENTIONS





        // ----------------
        //   INDICATORS
        // ----------------
        else if (mqtt_topic[0] == "indicators" && mqtt_topic[2] == "RLC")
        {
            /* USER LOCATION UPDATE
             *
            * JSON format for User location update:
            *  {
            *      "userid":“12345”
            *      "ivcode":“RLC”
            *      "time":{"temporality":"timestamp", "t":"1494256770.105"}
            *      "data":{"location":{"value":"room_label", "units":"code"} }
            *  }
            */

            std::string messageValue = new_mqtt2ros_msg->value;
            try{
                auto json_msg = json::parse( messageValue );
                if (verbose) ROS_INFO("[TaskCoordinator] New User Location received via MQTT: [%s]->%s\n", new_mqtt2ros_msg->key.c_str(),messageValue.c_str());

                // 1. Update User Location (if needed)
                if( json_msg["data"]["location"]["value"] != user_location && json_msg["data"]["location"]["value"] != "UNKNOWN")
                {
                    // User location has changed! Spread the word and inform other nodes about this!
                    if (oral_verbose) say("The VC has notified about a new user location");

                    user_location = json_msg["data"]["location"]["value"];
                    ROS_INFO("[TaskCoordinator] New User Location received via MQTT. User is at %s\n", user_location.c_str());
                    std_msgs::String user_location_msg;
                    user_location_msg.data = user_location;
                    user_expected_location_pub.publish(user_location_msg);


                    // Outdoor
                    if (outdoor_location)
                    {
                        if( json_msg["data"]["location"]["value"] == "OUTDOOR" ||
                            json_msg["data"]["location"]["value"] == "Outdoor" ||
                            json_msg["data"]["location"]["value"] == "outdoor" ||
                            json_msg["data"]["location"]["value"] == "OUTSIDE" ||
                            json_msg["data"]["location"]["value"] == "outside" ||
                            json_msg["data"]["location"]["value"] == "Outside")
                        {
                            // User has left the environment. Cancell all pending tasks and go Dock
                            //0. Fire request to cancel all the Tasks to Task_Manager (via service)
                            task_manager::removeTask task_r;
                            task_r.request.task_id = -1;
                            task_r.request.info = "Task cancelled because user location is set to OUTDOOR";
                            if(!task_manager_srv_remove.call(task_r))
                                ROS_ERROR("[TaskCoordinator] Failed to call service remove_task with task_id %i", task_r.request.task_id);

                            // 1. Command return to docking
                            // Get pose of the Docking station or quit if not found
                            if (!dock_station_found)
                            {
                               ROS_WARN("[TaskCoordinator] Docking_Station location not found in the map!. Skipping request.");
                               return;
                            }

                            // Fire request to the Task_Manager (via service)
                            task_manager::addTask task;
                            task.request.task_name = "dock";
                            task.request.task_priority = 8;                      // Highest priority task!
                            task.request.task_permanence = false;
                            task.request.task_impact = "cancel";                 // Cancel all pending tasks!
                            task.request.task_args.clear();
                            //params [docking_pose, speech]
                            task.request.task_args.push_back(dock_station.poses_str[0]);  // dock station pose
                            task.request.task_args.push_back("false");          // speech

                            // Execute task (do not keep track, as there is no intervention)
                            int assigned_task_id = sendTask(task);
                            if (assigned_task_id < 0)
                            {
                                ROS_WARN("[TaskCoordinator] Dock action has NOT been created.");
                                return;
                            }
                        }
                    }
                }

                // 2. Use this topic as a keep alive from VC (in theoury each 30sec)
                last_keepAlive = ros::Time::now();


            }catch (exception e)
            {
                ROS_ERROR("[taskCoordinator] Malformed JSON request on user location... ignoring it");
                return;
            }
        }// end indicators-RLC


        else
        {
            //Ignore
        }

    } catch(std::runtime_error& ex)
    {
        ROS_ERROR("[TaskCoordinator] Exception on mqtt2rosCallBack: [%s]", ex.what());
    }
}



//-----------------------------------------------------------------------------------
//                             TASK COMPLETED CALLBACK
//-----------------------------------------------------------------------------------
// A Task has been completed! Keep track and forward result through MQTT
/*
*  task_complemeted_msg = {
*      "task_id" : "ID from task manger",
*      "name" : "TASK_NAME",
*      "status" : "success/failure",
*      "duration" : "SEC",
*      "trace" : {"component":"description", "component":"description"}
* }
*/
void CTaskCoordinator::task_completedCB(const std_msgs::String::ConstPtr& task_complemeted_msg)
{
    try
    {
        auto json_msg = json::parse( task_complemeted_msg->data );
        std::string id = json_msg["task_id"];
        std::string name = json_msg["name"];
        std::string status = json_msg["status"];
        std::string duration_s = json_msg["duration"];
        float duration = atof(duration_s.c_str());
        std::string response;


        // A. UPDATE STATISTICS
        //----------------------
        update_statistics(name, status, duration );


        // B. REPORT THROUGH MQTT
        //-------------------------        
        std::map<int,string>::iterator it_task;
        it_task = tasks_in_execution.find( atoi(id.c_str()) );
        if (it_task == tasks_in_execution.end())
        {
            if (verbose) ROS_WARN("[TaskCoordinator] A task has ended, but it was not started by us..Doing nothing.");
            return;
        }

        // The finished task was created by an external request, reply with an ACK
        if (false) ROS_INFO("[TaskCoordinator] Task[%i] corresponds to intervention->%s", it_task->first, it_task->second.c_str());
        if (false) ROS_INFO("[TaskCoordinator] Task[%i] output is->%s", it_task->first, task_complemeted_msg->data.c_str());
        auto original_json = json::parse( it_task->second );
        if (oral_verbose) say("Intervention with code " + original_json["data"]["code"].get<std::string>() + " has been completed" );


        // OBJECT SEARCH
        // ----------------
        // OS ack is a bit different as it includes the object_id, response=found/notfound, location and room_id
        if (original_json["data"]["code"] == "OS")
        {
            try
            {
                // Create a JSON ACK
                /*
                 *  {
                 *      "id" : <ACK_ID>,
                 *      "intervention_id" : <UUID>,
                 *      "userid":<userid>,
                 *      "time":{"temporality":"timestamp", "t":<timestamp>},
                 *      "data":{
                 *              "code":"OS",
                 *              "objectid":"<string>",
                 *              "response":"Found/Not found",
                 *              "location":[x,y,phi],
                 *              "roomid":"<string>"
                 *              }
                 *      "trace":""
                 *  }
                 */

                // Check the response of the ObjectSearch action_server
                std::string object_id = "empty";
                std::string object_location = "empty";
                std::string object_room = "empty";
                response = "Not found";

                // result
                if (json_msg["trace"].find("OSresult") != json_msg["trace"].end())
                {
                    if( json_msg["trace"]["OSresult"] == "yes" )
                        response = "Found";
                    else
                        response = "Not found";
                }

                // object-id
                if (json_msg["trace"].find("objectid") != json_msg["trace"].end())
                    object_id = json_msg["trace"]["objectid"];
                else
                    object_id = original_json["data"]["objectid"];

                //object-location (x,y,phi)
                if (json_msg["trace"].find("objectlocation") != json_msg["trace"].end())
                    object_location = json_msg["trace"]["objectlocation"];

                //object-topological-location
                if (json_msg["trace"].find("objectspacelocation") != json_msg["trace"].end())
                    object_room = json_msg["trace"]["objectspacelocation"];


                if (false) ROS_INFO("[TaskCoordinator] Sending ACK");
                json ack;
                boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                std::string timestamp_readable = boost::posix_time::to_iso_extended_string(my_posix_time);
                ack["id"] = "giraffX-" + timestamp_readable;             // unique identifier
                ack["intervention_id"] = original_json["id"];
                ack["userid"] = original_json["userid"];
                ack["time"]["temporality"] = "timestamp";
                ack["time"]["t"] = boost::to_string( ros::Time::now() );
                ack["data"]["code"] = original_json["data"]["code"];
                ack["data"]["objectid"] = object_id;
                ack["data"]["response"] = response;
                ack["data"]["location"] = object_location;
                ack["data"]["roomid"] = object_room;
                ack["trace"] = json_msg["trace"];

                // Publish over MQTT
                diagnostic_msgs::KeyValue task_finished;
                task_finished.key = "ack/" + user_id + "/INFO";
                task_finished.value = ack.dump();
                mqtt_pub.publish(task_finished);
                if (verbose) ROS_INFO("[TaskCoordinator] A task has ended, sending ACK to VC [%s]-->%s", task_finished.key.c_str(), task_finished.value.c_str());
            }
            catch (exception e)
            {
                ROS_ERROR("[taskCoordinator - send_ack] Malformed JSON on ACK to MQTT - %s", e.what());
                return;
            }
        }


        // STANDARD ACK
        // ----------------
        else
        {
            if (json_msg["status"] == "failure")
            {
                //Some component failed! report error and in the trace field, add the description of the error cause.
                response = "Error";
            }
            else
            {
                // Action was completed! Success
                // Forward the response of the HRI action_server (if any)
                if (json_msg["trace"].find("HRI") != json_msg["trace"].end())
                    response = json_msg["trace"]["HRI"];
                else
                    response = "OK";    //default success response
            }


            if (original_json["data"]["code"] == "CH")
            {
                // Careful: trace is not just a string, its a json object
                send_ack("indicators", original_json["ivcode"], original_json["id"], original_json["userid"], original_json["data"]["code"], response, json_msg["trace"].dump() );
            }
            else
            {
                // Carefull: trace is not just a string, its a json object
                send_ack("ack", original_json["ivcode"], original_json["id"], original_json["userid"], original_json["data"]["code"], response, json_msg["trace"].dump() );
            }

        }


        // C. CHECK IF CALL FOR HELP
        // Deactivate the status to allow new HELP triggers.
        if (original_json["data"]["code"] == "CH")
            alert_active = false;

        // D. CHECK IF TELEOPERATION
        if (original_json["data"]["code"] == "TO")
            teleop_active = false;

        // E. REMOVE TASK FROM active task list
        tasks_in_execution.erase(it_task);
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator - task_completedCB] Malformed JSON on ACK to MQTT - %s", e.what());
        ROS_ERROR("[taskCoordinator - task_completedCB] %s", task_complemeted_msg->data.c_str());
        return;
    }
}



//-----------------------------------------------------------------------------------
//                                  SEND_ACK (ROS --> MQTT)
//-----------------------------------------------------------------------------------
// A Task has been accepted/completed/cancelled: Keep track and forward result through MQTT
void CTaskCoordinator::send_ack(std::string topic, std::string subtopic, std::string intervention_id, std::string user_id, std::string intervention_code, std::string response, std::string trace)
{
    try
    {
        // Create a JSON ACK
        /*
         *  {
         *      "id" : <ACK_ID> unique identifier
         *      "intervention_id" : <UUID>,
         *      "userid":<userid>,
         *      "time":{"temporality":"timestamp", "t":<timestamp>},
         *      "data":{"code":"XX", "response":"OK/Later/Error/Yes/No/etc" },
         *      "trace":""
         *  }
         *
         */

        json ack;

        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        std::string timestamp_readable = boost::posix_time::to_iso_extended_string(my_posix_time);

        ack["id"] = "giraffX-" + timestamp_readable;             // unique identifier
        ack["intervention_id"] = intervention_id;               // id of the former request, to keep track
        ack["userid"] = user_id;
        ack["time"]["temporality"] = "timestamp";
        ack["time"]["t"] = boost::to_string( ros::Time::now() );
        ack["data"]["code"] = intervention_code;
        ack["data"]["response"] = response;
        auto trace_json = json::parse( trace );
        ack["trace"] = trace_json;
        if (topic == "indicators" && subtopic == "HELP")
            ack["icode"] = subtopic;

        //ROS_INFO( "[TaskCoordinator] Adding Trace, with value [%s]", trace.c_str() );
        //ROS_INFO( "[TaskCoordinator] Adding response with value [%s]", response.c_str() );
        //ROS_INFO( "[TaskCoordinator] Adding code with value [%s]", intervention_code.c_str() );
        //ROS_INFO( "[TaskCoordinator] ACK ready!");

        // Publish over MQTT
        diagnostic_msgs::KeyValue task_finished;
        task_finished.key = topic +"/" + user_id + "/" + subtopic;
        task_finished.value = ack.dump();
        mqtt_pub.publish(task_finished);
        if (verbose) ROS_INFO("[TaskCoordinator] A task has ended, sending ACK to VC [%s]-->%s", task_finished.key.c_str(), task_finished.value.c_str());
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator - send_ack] Malformed JSON on ACK to MQTT - %s", e.what());
        return;
    }
}



//-----------------------------------------------------------------------------------
//                              LOAD TASK STATISTICS FROM FILE
//-----------------------------------------------------------------------------------
void CTaskCoordinator::load_task_statistics()
{
   if (verbose) ROS_INFO("[TaskCoordinator] Loading task statistics from file (if available)");

    tasks_statistics.clear();

    //Check if file exists with statistics from previous executions
    ifstream myfile (task_statistics_file_path.c_str());
    if (myfile.is_open())
    {
        std::string line;
        bool first_line = true;
        while ( std::getline(myfile,line) )
        {
            if (first_line)
            {
                first_line = false;
                continue;
            }

            // Parse line
            std::string delimiter = ", ";
            size_t pos = 0;
            std::string token[7];
            size_t i = 0;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                token[i] = line.substr(0, pos);
                line.erase(0, pos + delimiter.length());
                i++;
            }
            token[i] = line;

            // Add task to task_statistics map
            Ttask_statistics new_task;
            new_task.num_runs = atoi(token[1].c_str());
            new_task.num_success = atoi(token[2].c_str());
            new_task.num_failures = atoi(token[3].c_str());
            new_task.avg_exec_time = atof(token[4].c_str());
            new_task.var_exec_time = atof(token[5].c_str());
            new_task.success_rate = atof(token[6].c_str());
            tasks_statistics.insert( std::pair<std::string, Ttask_statistics>(token[0], new_task) );
        }
        myfile.close();
    }
    else
        if (verbose) ROS_INFO("[TaskCoordinator] No initial task statistics found... Starting new statistics");
}


//-----------------------------------------------------------------------------------
//                              UPDATE TASK STATISTICS
//-----------------------------------------------------------------------------------
void CTaskCoordinator::update_statistics( std::string task_name, std::string status, float duration )
{
    // If task is navigation to WP (GO_TO_WP_x) from the PATROL task, keep record to get statistics from point to point navigation
    if ( task_name.find("GO_TO_WP_") == 0 )
    {
        //update current robot location and task name
        std::string new_robot_location = task_name.substr(6);   //The WP_x number
        task_name = "GO_FROM_" + robot_location + "_TO_" + new_robot_location;
        robot_location = new_robot_location;
    }

    // Check if finished task is being already monitored
    std::map<std::string, Ttask_statistics>::iterator it;
    it = tasks_statistics.find( task_name );
    if (it == tasks_statistics.end())
    {
        // New task to monitor (add to list)
        Ttask_statistics new_task;
        new_task.num_runs = 1;

        if( (status == "SUCCESS") || (status == "success") || (status == "OK") || (status == "ok"))
        {
            new_task.num_success = 1;
            new_task.num_failures = 0;
            new_task.success_rate = 1.0;
            //control times only for successs runs
            new_task.avg_exec_time = duration;
            new_task.var_exec_time = 0.0;
        }
        else
        {
            new_task.num_success = 0;
            new_task.num_failures = 1;
            new_task.success_rate = 0.0;
            new_task.avg_exec_time = 0.0;
            new_task.var_exec_time = 0.0;
        }
        tasks_statistics.insert( std::pair<std::string, Ttask_statistics>(task_name, new_task) );
    }
    else
    {
        // Task already monitored. Update Statistics
        it->second.num_runs += 1;
        if( (status == "SUCCESS") || (status == "success") || (status == "OK") || (status == "ok"))
        {
            it->second.num_success += 1;
            // Update execution time (mean and variance) only for success attemps
            float N = (float)it->second.num_success;
            if (N > 1)
            {
                float avg_duration_old = it->second.avg_exec_time;
                it->second.avg_exec_time = (1/N) * (avg_duration_old*(N-1) + duration);
                it->second.var_exec_time = (1/(N-1)) * ( (N-2)*it->second.var_exec_time + (duration-it->second.avg_exec_time)*(duration-avg_duration_old) );
            }
            else
            {
                it->second.avg_exec_time = duration;
                it->second.var_exec_time = 0.0;
            }
        }
        else
        {
            it->second.num_failures += 1;
            // TODO --> Add robot location (x,y,phi) to point problematic areas in the map
        }
        it->second.success_rate = (float)it->second.num_success / (float)it->second.num_runs;
    }


    // Save current status file to keep a persistent copy
    // CSV -> Task_name,  num_runs, num_success, num_failures, avg_exec_time, var_exec_time, success_rate
    ofstream myfile;
    if (false) ROS_INFO("[TaskCoordinator] Writting task statistics to file: [%s]", task_statistics_file_path.c_str());
    myfile.open (task_statistics_file_path);
    myfile << "# Task_name,  num_runs, num_success, num_failures, avg_exec_time, var_exec_time, success_rate \n";
    for (std::map<std::string, Ttask_statistics>::iterator it=tasks_statistics.begin(); it!=tasks_statistics.end(); ++it)
    {
        myfile << it->first << ", " << it->second.num_runs << ", " << it->second.num_success<< ", " << it->second.num_failures << ", ";
        myfile << it->second.avg_exec_time<< ", " << it->second.var_exec_time<< ", " << it->second.success_rate << '\n';
    }
    myfile.close();
}



//-----------------------------------------------------------------------------------
//                                   Giraff_buttonsCB
//-----------------------------------------------------------------------------------
void CTaskCoordinator::giraff_buttonsCB(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    // New button has been pressed. Format is (Joy.buttons = Red, Green, Dial, E-Stop)
    // We are only interested in the E-Stop
    if (!e_stop_active && joy_msg->buttons[3] == 1)
    {
        // E-STOP has been pressed!! Fire request to cancel all the Tasks to Task_Manager (via service)
        task_manager::removeTask task_r;
        task_r.request.task_id = -1;
        task_r.request.info = "Task cancelled by E-STOP button press";
        if(!task_manager_srv_remove.call(task_r))
            ROS_ERROR("[TaskCoordinator] Failed to call service remove_task with task_id %i", task_r.request.task_id);

        // Open GUI to inform the user how to restore normal functionality
        e_stop_active = true;
        //update_interface_web();

        // Command a SAY task to inform the user.
        task_manager::addTask task;
        task.request.task_name = "say";
        task.request.task_priority = 5;
        task.request.task_permanence = false;
        task.request.task_impact = "none";
        task.request.task_args.clear();
        //params (text)
        if (user_language == "ES")
            task.request.task_args.push_back("El botón de emergencia ha sido activado. Cambiando a modo pasivo.");
        else if (user_language == "IT")
            task.request.task_args.push_back("Il pulsante di emergenza è stato attivato. Entrando in modalità passiva.");
        else // default is "EN"
            task.request.task_args.push_back("The emergency button has been activated. Entering passive mode.");
        sendTask(task);

        //publish emergency status
        std_msgs::Bool estop;
        estop.data = e_stop_active;
        estop_pub.publish(estop);
    }
    else if (e_stop_active && joy_msg->buttons[3] == 0)
    {
        //E-STOP has been released
        // Close e-stop GUI
        e_stop_active = false;
        //update_interface_web();
        
        // Command a SAY task to inform the user.
        task_manager::addTask task;
        task.request.task_name = "say";
        task.request.task_priority = 5;
        task.request.task_permanence = false;
        task.request.task_impact = "none";
        task.request.task_args.clear();
        //params (text)
        if (user_language == "ES")
            task.request.task_args.push_back("El botón de emergencia ha sido liberado. Cambiando a modo de funcionamiento normal.");
        else if (user_language == "IT")
            task.request.task_args.push_back("Il pulsante di emergenza è stato rilasciato. Entrare nella modalità di funzionamento normale.");
        else // default is "EN"
            task.request.task_args.push_back("The emergency button has been released. Entering normal operation mode.");
        sendTask(task);
        
        //publish emergency status
        std_msgs::Bool estop;
        estop.data = e_stop_active;
        estop_pub.publish(estop);
    }
}

/*
void CTaskCoordinator::update_interface_web()
{
    // Ensure the correct web-page is opened.
    if (e_stop_active)
    {
        // e-stop has highest priority.
        // Open e-stop page - Local webs are hosted in movecare_interface/GUI/static/web/
        interface_srv_call("open", "static/web/e_stop_gui_" + user_language + ".html");
    }
    else if (offline_warning_active)
    {
        // Offline has low priority.
        // Open offline page
        interface_srv_call("open", "static/web/offline_gui_" + user_language + ".html");
    }
    else
    {
        // Nothing to display. Close all to ensure we show the default interface
        interface_srv_call("close", "");
    }
}
*/


inline bool file_exists (const std::string& name)
{
    if (FILE *file = fopen(name.c_str(), "r"))
    {
        fclose(file);
        return true;
    }
    else
        return false;
}



//-----------------------------------------------------------------------------------
//                           Get Data from file
//-----------------------------------------------------------------------------------
json CTaskCoordinator::get_data_from_file(std::string path, bool &success)
{
    if (verbose) ROS_INFO("\n[TaskCoordinator] Reading config from file: %s\n", path.c_str());

    // read a JSON file
    if ( file_exists(path) )
    {
        std::ifstream ifs(path);
        json json_data = json::parse(ifs);
        success = true;
        return json_data;
    }
    else
    {
        ROS_ERROR( "\n[TaskCoordinator] MISSING config file: %s\n", path.c_str() );
        json empty_data;
        success = false;
        return empty_data;
    }
}


//-----------------------------------------------------------------------------------
//                           Get Data from URL
//-----------------------------------------------------------------------------------
json CTaskCoordinator::get_data_from_url(std::string url, bool &success)
{
    ROS_INFO("[TaskCoordinator] Loading user info from url: %s\n", url.c_str());
    success = false;

    // Use CURL to get data from HGW server (Json data)
    CURL *conn = NULL;
    CURLcode code;
    curl_global_init(CURL_GLOBAL_DEFAULT);

    // Initialize CURL connection
    /*
    if( !init_curl(conn, url) )
    {
        ROS_ERROR("[TaskCoordinator] Curl Connection initializion failed");
        exit(EXIT_FAILURE);
    }
    */
    // Retrieve content from the URL
    code = curl_easy_perform(conn);
    curl_easy_cleanup(conn);

    if(code != CURLE_OK)
    {
        ROS_ERROR("[TaskCoordinator] CURL Failed to easy_perform [%s]\n", errorBuffer);
        exit(EXIT_FAILURE);
    }

    // Parse content (BUFFER)
    json json_msg;
    try
    {
        json_msg = json::parse( curl_buffer );
        success = true;
        curl_buffer.clear();
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator] Error parsing JSON data from URL %s. Error:  [%s]", url.c_str(), e.what());
        ROS_ERROR("[taskCoordinator] CURL_BUFFER: ");
        std::cout << curl_buffer << std::endl;
        ROS_ERROR("[taskCoordinator] json_msg parsed (with error): ");
        std::cout << json_msg.dump(4) << std::endl;
        success = false;
    }

    return json_msg;
}


//-----------------------------------------------------------------------------------
//                         Get User Data from JSON msg
//-----------------------------------------------------------------------------------
void CTaskCoordinator::get_user_data_from_json(json json_msg, bool &success)
{
    /*
    {
        "uuid":"2c938084683d9f87016bc28ff1390093",
        "username": "user0000007",
        "email": "user0000007@mail.com",
        "attributes": {
            "firstname": "some value",
            "lastname": "fffdf",
            "language": "ES",
            "home-config":" .... ....."
        },
        "roles": [
          "a73611aab8de11e781af0242ac120002"
        ],
        "enable":1
    }
   */
    if (verbose) ROS_INFO("\n[TaskCoordinator] PARSING USER CONFIG FROM  --- JSON ---");
    if (verbose) ROS_INFO("\n[TaskCoordinator] JSON IS:\n");
    if (verbose) std::cout << json_msg.dump(4) << std::endl;

    try
    {
        user_name = json_msg["attributes"]["firstname"];
    }
    catch (exception e)
    {
        ROS_WARN("[taskCoordinator] Error loading User data. {firstname} parameter not set in the JSON msg. Using default. Error:  [%s]", e.what());
        //std::cout << json_msg.dump(4) << std::endl;
        user_name = "NOT_SET";
        success = false;
        return;
    }

    try
    {
        user_surname = json_msg["attributes"]["lastname"];
    }
    catch (exception e)
    {
        ROS_WARN("[taskCoordinator] Error loading User data. {lastname} parameter not set in the JSON msg. Using default. Error: [%s]", e.what());
        //std::cout << json_msg.dump(4) << std::endl;
        user_surname = "NOT_SET";
        success = false;
        return;
    }

    try
    {
        user_language = json_msg["attributes"]["language"];

        // Ensure correct format: EN/ES/IT
        if (user_language=="es" || user_language=="Es" || user_language=="spanish" || user_language=="esp")
            user_language = "ES";

        if (user_language=="it" || user_language=="It" || user_language=="ita" || user_language=="italian")
            user_language = "IT";

        if (user_language=="en" || user_language=="En" || user_language=="english" || user_language=="eng")
            user_language = "EN";
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator] Error loading User data. {language} parameter not set in the JSON msg. Using default. Error: [%s]", e.what());
        //std::cout << json_msg.dump(4) << std::endl;
        user_language = "NOT_SET";
        success = false;
        return;
    }

    // Reaching this points means all data is ok
    success = true;
    return;
}



//-----------------------------------------------------------------------------------
//                         Get Env Data from JSON msg
//-----------------------------------------------------------------------------------
void CTaskCoordinator::get_env_data_from_json(json json_msg, bool &success)
{
    /*
     * The JSON floorplan looks like this:
    {
        "passages":[
            {"location":{"y":-0.976899125,"x":1.232270875},"label":"PASS93","connecting":["HALLWAY1","HALLWAY2"]},
            {"location":{"y":1.675,"x":-1.9},"label":"PASS59","connecting":["HALLWAY1","HALLWAY3"]},
            {"location":{"y":4.117333125,"x":1.39902625},"label":"PASS105","connecting":["HALLWAY2","HALLWAY3"]},
            {"location":{"y":-1.61433725,"x":-2.570976875},"label":"PASS55","connecting":["OUTSIDE1","HALLWAY1"]}
        ],
        "format":2,
        "poses":[
            {"space_label":"HALLWAY3","location":{"y":3.569545,"x":-0.689465875}},
            {"space_label":"HALLWAY3","location":{"y":7.5355625,"x":-0.0051883124999961}},
            {"space_label":"HALLWAY3","location":{"y":5.91564,"x":-2.3093885625}},
            {"space_label":"HALLWAY1","location":{"y":0.231946275,"x":-1.9044076875}},
            {"space_label":"HALLWAY3","location":{"y":2.41046275,"x":-3.077455}},
            {"space_label":"HALLWAY2","location":{"y":2.927162,"x":2.634168125}}
        ],
        "setup":[
            {"location_label":"HALLWAY3","location_type":"space","sensor_id":"ZB-000D6F00110886C1"},
            {"location_label":"HALLWAY1","location_type":"space","sensor_id":"ZB-000D6F0011088C2E"},
            {"location_label":"TV1","location_type":"object","sensor_id":"ZB-000D6F000D3D908A"},
            {"location_label":"HALLWAY3","location_type":"space","sensor_id":"ZB-000D6F00118C3BB6"},
            {"location_label":"HALLWAY3","location_type":"space","sensor_id":"BLE-001304195F58"}
        ],
        "sensors":[
            {"mcodes":["DMV"],"id":"ZB-000D6F00110886C1"},
            {"mcodes":["PTB"],"id":"BLE-247189D00402"},
            {"mcodes":["DMV"],"id":"ZB-000D6F0011087012"},
            {"mcodes":["DMV"],"id":"ZB-000D6F0011088C2E"}
        ],
        "spaces":[
            {"category":"outside","label":"OUTSIDE1","location":{"y":-1.45,"x":-3.775}},
            {"category":"hallway","label":"HALLWAY2","location":{"y":2.1041490625,"x":1.753681}},
            {"category":"hallway","label":"HALLWAY3","location":{"y":4.108594125,"x":-2.09365275}},
            {"category":"hallway","label":"HALLWAY1","location":{"y":-1.437535125,"x":-1.262535875}}
        ],
        "userid":"2c938084683d9f87016a92216a6f005e",
        "objects":[
            {"type":"tv","location":{"y":3.06298875,"x":-4.101639375},"label":"TV1","space_label":"HALLWAY3"},
            {"type":"docking_station","location":{"y":-0.2,"x":-0.9625},"label":"docking_station","space_label":"HALLWAY1"}
        ]
    }
    */

    if (verbose) ROS_INFO("\n[TaskCoordinator] PARSING FLOORPLAN FROM  --- JSON ---");
    if (verbose) ROS_INFO("\n[TaskCoordinator] JSON IS:\n");
    if (verbose) std::cout << json_msg.dump(4) << std::endl;

    //1. Get "userid"
    try
    {
        user_id = json_msg["userid"];
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator] Error loading User data. userid parameter not set in the JSON msg. [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        user_id = "NOT_SET";
        success = false;
        return;
    }

    /*
     *
     *  TODO: Replace this code by a call to the GRAPH node to get the spaces/passages/docking in the environment
     *
     *
     */


    //1.1 Do we have "poses" (old/new version)
    bool have_poses = false;
    try
    {
        if (json_msg.find("poses") != json_msg.end())
            have_poses = true;
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator] Error getting Poses. Error is: [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        success = false;
        return;
    }

    //2. Load array of spaces/locations/rooms
    try
    {
        locations.clear();
        char buffer [50];
        for (json::iterator it = json_msg["spaces"].begin(); it != json_msg["spaces"].end(); ++it)
        {
            auto content = it.value();
            std::string category = content["category"];

            // Ignore Outside spaces as the robot should never attempt to go there.
            if (category != "OUTSIDE" && category != "Outside" && category != "outside")
            {
                std::string label = content["label"];
                Tlocation new_loc;

                if (!have_poses)
                {
                    //Single pose per space
                    geometry_msgs::Pose pose;
                    if (content["location"].find("yaw") != content["location"].end())
                    {
                        pose.position.x = content["location"]["x"];
                        pose.position.y = content["location"]["y"];
                        pose.position.z = 0.0;
                        pose.orientation = tf::createQuaternionMsgFromYaw(content["location"]["yaw"]);

                        sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, (double)content["location"]["yaw"]);
                    }
                    else
                    {
                        pose.position.x = content["location"]["x"];
                        pose.position.y = content["location"]["y"];
                        pose.position.z = 0.0;
                        pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

                        sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, 0.0);
                    }
                    std::string pose_str(buffer);
                    new_loc.poses.push_back(pose);
                    new_loc.poses_str.push_back(pose_str);
                }
                else
                {
                    new_loc.poses.clear();
                    new_loc.poses_str.clear();
                }

                // Add new Space
                locations.insert( std::pair<string,Tlocation>(label, new_loc) );
                if (verbose) ROS_INFO("[TaskCoordinator] New room with label = %s contains %lu poses", label.c_str(), new_loc.poses.size());
            }
        }
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator] Error loading Env data. Spaces list malformed [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        success = false;
        return;
    }


    if (have_poses)
    {
        //2.1 Load array of poses
        try
        {
            char buffer [50];
            for (json::iterator it = json_msg["poses"].begin(); it != json_msg["poses"].end(); ++it)
            {
                auto content = it.value();
                std::string space_label = content["space_label"];

                // Ignore Outside related poses as the robot should never attempt to go there.
                if (space_label != "OUTSIDE" && space_label != "Outside" && space_label != "outside")
                {
                    geometry_msgs::Pose pose;
                    if (content["location"].find("yaw") != content["location"].end())
                    {
                        pose.position.x = content["location"]["x"];
                        pose.position.y = content["location"]["y"];
                        pose.position.z = 0.0;
                        pose.orientation = tf::createQuaternionMsgFromYaw(content["location"]["yaw"]);

                        sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, (double)content["location"]["yaw"]);
                    }
                    else
                    {
                        pose.position.x = content["location"]["x"];
                        pose.position.y = content["location"]["y"];
                        pose.position.z = 0.0;
                        pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

                        sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, 0.0);
                    }
                    std::string pose_str(buffer);

                    // Add new pose to existing Space
                    std::map<string,Tlocation>::iterator it_space;
                    it_space = locations.find(space_label);
                    if (it_space != locations.end())
                    {
                        it_space->second.poses.push_back(pose);
                        it_space->second.poses_str.push_back(pose_str);
                        if (verbose) ROS_INFO("[TaskCoordinator] New pose for Space %s with value %s", space_label.c_str(), pose_str.c_str());
                    }

                }
            }
        }
        catch (exception e)
        {
            ROS_ERROR("[taskCoordinator] Error loading Env data. Spaces list malformed [%s]", e.what());
            std::cout << json_msg.dump(4) << std::endl;
            success = false;
            return;
        }
    }

    //3. Load array of passages/doors
    try
    {
        passages.clear();
        char buffer [50];
        for (json::iterator it = json_msg["passages"].begin(); it != json_msg["passages"].end(); ++it)
        {
            auto content = it.value();
            std::string label = content["label"];
            geometry_msgs::Pose pose;

            // find if YAW is set or not
            if (content["location"].find("yaw") != content["location"].end())
            {
                pose.position.x = content["location"]["x"];
                pose.position.y = content["location"]["y"];
                pose.position.z = 0.0;
                pose.orientation = tf::createQuaternionMsgFromYaw(content["location"]["yaw"]);

                sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, (double)content["location"]["yaw"]);
            }
            else
            {
                pose.position.x = content["location"]["x"];
                pose.position.y = content["location"]["y"];
                pose.position.z = 0.0;
                pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

                sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, 0.0);
            }
            std::string pose_str(buffer);

            Tpassage new_pass;
            new_pass.pose_str = pose_str;
            new_pass.pose = pose;
            passages.insert( std::pair<string,Tpassage>(label, new_pass) );
            if (verbose) ROS_INFO("[TaskCoordinator] New passage with label = %s", label.c_str());
            if (verbose) ROS_INFO("[TaskCoordinator]                  pose = %s", pose_str.c_str());

            // Match passages with spaces
            for (json::iterator it2 = content["connecting"].begin(); it2 != content["connecting"].end(); ++it2)
            {
                std::string room_id = it2.value();
                // find the space with this name, and add the passage name
                std::map<string,Tlocation>::iterator it_room;
                it_room = locations.find(room_id);
                if (it_room != locations.end())
                    it_room->second.passages.push_back(label);
            }

        }//end-for each passage
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator] Error loading Env data. Passages list malformed [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        success = false;
        return;
    }


    // 4. Check Objects (To get docking_station navigation pose)
    try
    {
        char buffer [50];
        for (json::iterator it = json_msg["objects"].begin(); it != json_msg["objects"].end(); ++it)
        {
            auto content = it.value();
            if (content["label"] == "docking_station" || content["type"]=="dockingstation")
            {
                dock_station_found = true;

                //Get pose
                geometry_msgs::Pose pose;

                // find if YAW is set or not
                if (content["location"].find("yaw") != content["location"].end())
                {
                    pose.position.x = content["location"]["x"];
                    pose.position.y = content["location"]["y"];
                    pose.position.z = 0.0;
                    pose.orientation = tf::createQuaternionMsgFromYaw(content["location"]["yaw"]);

                    sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, (double)content["location"]["yaw"]);
                }
                else
                {
                    pose.position.x = content["location"]["x"];
                    pose.position.y = content["location"]["y"];
                    pose.position.z = 0.0;
                    pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

                    sprintf(buffer, "[%.3f, %.3f, %.3f]", pose.position.x, pose.position.y, 0.0);
                }
                std::string pose_str(buffer);
                dock_station.poses.push_back(pose);
                dock_station.poses_str.push_back(pose_str);
                dock_station.passages.clear();

                if (verbose) ROS_INFO("[TaskCoordinator] Docking Station object found.");
                if (verbose) ROS_INFO("[TaskCoordinator]               pose = %s", pose_str.c_str());

                break;
            }
        }
    }
    catch (exception e)
    {
        ROS_ERROR("[taskCoordinator] Error loading Env data. Passages list malformed [%s]", e.what());
        std::cout << json_msg.dump(4) << std::endl;
        success = false;
        return;
    }

    // Reaching this points means all data is ok
    if (dock_station_found)
        success = true;
    else
        success = false;

    return;
}


// -------------------------------------
//  libcurl write callback function
// -------------------------------------
int CTaskCoordinator::writer(char *data, size_t size, size_t nmemb, std::string *writerData)
{
  if(writerData == NULL)
    return 0;

  writerData->append(data, size*nmemb);

  return size * nmemb;
}


// -------------------------------------
//  libcurl connection initialization
// -------------------------------------
/*
bool CTaskCoordinator::init_curl(CURL *&conn, std::string url)
{
  CURLcode code;

  conn = curl_easy_init();
  if(conn == NULL) {
    ROS_ERROR("[TaskCoordinator] Failed to create CURL connection");
    exit(EXIT_FAILURE);
  }

  code = curl_easy_setopt(conn, CURLOPT_ERRORBUFFER, errorBuffer);
  if(code != CURLE_OK) {
    ROS_ERROR("[TaskCoordinator] Failed to set error buffer [%d]", code);
    return false;
  }

  code = curl_easy_setopt(conn, CURLOPT_URL, url.c_str());
  if(code != CURLE_OK) {
    ROS_ERROR("[TaskCoordinator] Failed to set URL [%s]", errorBuffer);
    return false;
  }

  code = curl_easy_setopt(conn, CURLOPT_FOLLOWLOCATION, 1L);
  if(code != CURLE_OK) {
    ROS_ERROR("[TaskCoordinator] Failed to set redirect option [%s]", errorBuffer);
    return false;
  }

  code = curl_easy_setopt(conn, CURLOPT_WRITEFUNCTION, writer);
  if(code != CURLE_OK) {
    ROS_ERROR("[TaskCoordinator] Failed to set writer [%s]", errorBuffer);
    return false;
  }

  code = curl_easy_setopt(conn, CURLOPT_WRITEDATA, &curl_buffer);
  if(code != CURLE_OK) {
    ROS_ERROR("[TaskCoordinator] Failed to set write data [%s]", errorBuffer);
    return false;
  }

  return true;
}
*/



// -------------------------------------
//  MARKERS
// -------------------------------------
void CTaskCoordinator::publish_room_markers()
{
    //For each location/rooom of the environment, publish a marker+label
    visualization_msgs::MarkerArray room_markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "rooms";
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    int i = 0;


    // SPACES - ROOMS
    for (std::map<string,Tlocation>::iterator it=locations.begin(); it!=locations.end(); it++)
    {
        // multiple poses are allowed
        for (std::vector<geometry_msgs::Pose>::iterator it2=it->second.poses.begin(); it2!=it->second.poses.end(); it2++)
        {
            marker.pose = *it2;

            if (it->first == user_location)
            {
                // Expected user location, Set GREEN marker
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else
            {
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }

            //1. Publish sphere
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.id = i;
            i++;
            // Add to array
            room_markers.markers.push_back(marker);


            // 2. Label: it->first
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = it->first;
            marker.id = i;
            i++;
            marker.pose.position.z = 0.2;
            // Add to array
            room_markers.markers.push_back(marker);
        }
    }


    // PASSAGES - DOORS
    for (std::map<string,Tpassage>::iterator it=passages.begin(); it!=passages.end(); it++)
    {
        marker.pose = it->second.pose;

        if (it->first == user_location)
        {
            // Expected user location, Set GREEN marker
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }

        //1. Publish sphere
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.id = i;
        i++;
        // Add to array
        room_markers.markers.push_back(marker);


        // 2. Label: it->first
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = it->first;
        marker.id = i;
        i++;
        marker.pose.position.z = 0.2;
        // Add to array
        room_markers.markers.push_back(marker);
    }


    // Docking Station (object)
    marker.pose = dock_station.poses[0];

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.2;

    //Publish sphere
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = i;
    i++;
    // Add to array
    room_markers.markers.push_back(marker);


    // 2. Label
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "dock_station";
    marker.id = i;
    i++;
    marker.pose.position.z = 0.2;
    // Add to array
    room_markers.markers.push_back(marker);

    marker_pub.publish(room_markers);
}


// -------------------------------------
//  KEEP ALIVE
// -------------------------------------
void CTaskCoordinator::check_conectivity()
{
    if ( !offline_warning_active && (ros::Time::now()-last_keepAlive) >= ros::Duration(120.0) )
    {
        // Something is not righ! Maybe internet connection problems. Notify the user.
        // Open GUI to inform the user
        offline_warning_active = true;
        //update_interface_web();

        // Command a SAY task to inform the user.
        /*
        task_manager::addTask task;
        task.request.task_name = "say";
        task.request.task_priority = 5;
        task.request.task_permanence = false;
        task.request.task_impact = "none";
        task.request.task_args.clear();
        //params (text)
        if (user_language == "ES")
            task.request.task_args.push_back("El botón de emergencia ha sido activado. Cambiando a modo pasivo.");
        else if (user_language == "IT")
            task.request.task_args.push_back("Il pulsante di emergenza è stato attivato. Entrando in modalità passiva.");
        else // default is "EN"
            task.request.task_args.push_back("The emergency button has been activated. Entering passive mode.");
        sendTask(task);
        */
    }
    else if (offline_warning_active && (ros::Time::now()-last_keepAlive) < ros::Duration(5.0) )
    {
        //Connection has been restablished
        // Close offline_warning GUI
        offline_warning_active = false;
        //update_interface_web();

        // Command a SAY task to inform the user.
        /*
        task_manager::addTask task;
        task.request.task_name = "say";
        task.request.task_priority = 5;
        task.request.task_permanence = false;
        task.request.task_impact = "none";
        task.request.task_args.clear();
        //params (text)
        if (user_language == "ES")
            task.request.task_args.push_back("El botón de emergencia ha sido liberado. Cambiando a modo de funcionamiento normal.");
        else if (user_language == "IT")
            task.request.task_args.push_back("Il pulsante di emergenza è stato rilasciato. Entrare nella modalità di funzionamento normale.");
        else // default is "EN"
            task.request.task_args.push_back("The emergency button has been released. Entering normal operation mode.");
        sendTask(task);
        */
    }
}

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_coordinator_node");

    CTaskCoordinator myTaskCoordinator;

    //Main Loop
    //----------
    ROS_INFO("[TaskCoordinator] Initialization complete... Looping");
    ros::Rate loop_rate(10);
    while( ros::ok() )
    {
        //1. Check for new interventions and attend CallBacks
        ros::spinOnce();

        //2. Check conectivity with VC
        myTaskCoordinator.check_conectivity();

        // myTaskCoordinator.publish_room_markers();
        loop_rate.sleep();
    }
    return(0);
}
