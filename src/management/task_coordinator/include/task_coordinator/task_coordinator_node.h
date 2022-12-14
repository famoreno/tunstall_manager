/** ****************************************************************************************
*  This node implements an API to control and manage the robots of the MAPIR group.
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
******************************************************************************************** */

#ifndef CTaskCoordinator_H
#define CTaskCoordinator_H

#include <ros/ros.h>
#include <ros/package.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//service messages from Task_Manager
#include <task_manager/addTask.h>
#include <task_manager/removeTask.h>

#include <iostream>
#include <fstream>
#include <curl/curl.h>
//boost
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/exception/to_string.hpp>
//json
#include <json.hpp>
using json = nlohmann::json;


struct Ttask_statistics
{
    Ttask_statistics() : num_runs(0), num_success(0), num_failures(0), avg_exec_time(0.0), var_exec_time(0.0), success_rate(0.0) {}
    size_t num_runs;
    size_t num_success;
    size_t num_failures;
    float avg_exec_time;
    float var_exec_time;
    float success_rate;
};


struct Tlocation
{
    std::vector<geometry_msgs::Pose>        poses;          // One Location can set multiple poses
    std::vector<std::string>                poses_str;
    std::vector<std::string>                passages;       // One location may have multiple passages to other locations
};

struct Tpassage
{
    geometry_msgs::Pose         pose;
    std::string                 pose_str;
};


class CTaskCoordinator
{
public:
    CTaskCoordinator();
    ~CTaskCoordinator();

    std::string input_topic;
    std::string output_topic;
    std::string buttons_topic;
    bool verbose, oral_verbose, outdoor_location;
    void publish_room_markers();
    void check_conectivity();

protected:
    ros::NodeHandle n;

    // Subscriptions & Publishers
    ros::Subscriber mqtt_sub, task_manager_results_sub, giraff_buttons_sub;
    ros::Publisher mqtt_pub, user_expected_location_pub, marker_pub, estop_pub, ready_pub;
    ros::ServiceClient task_manager_srv_add, task_manager_srv_remove, gui_srv_url;


    // CallBacks
    void mqtt2rosCallBack(const diagnostic_msgs::KeyValue::ConstPtr& new_mqtt2ros_msg);
    void task_completedCB(const std_msgs::String::ConstPtr& task_complemeted_msg);
    void giraff_buttonsCB(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void send_ack(std::string topic, std::string subtopic, std::string intervention_id, std::string user_id, std::string intervention_code, std::string response, std::string trace);

    //Functions
    inline int sendTask( task_manager::addTask &task );
    void say(std::string s);
    void update_statistics( std::string task_name, std::string status, float duration );
    void load_task_statistics();
    json get_data_from_url(std::string url, bool &success);
    json get_data_from_file(std::string path, bool &success);
    void get_env_data_from_json(json json_msg, bool &success);
    void get_user_data_from_json(json json_msg, bool &success);
    void replace_user_id(std::string &s);
    //void update_interface_web();
    void interface_srv_call(std::string action, std::string url);


    //Variables
    bool have_spaces;                                   // Avoids multiple request to get the spaces
    Tlocation dock_station;                             // Location of the docking station (set as a special location-object)
    bool dock_station_found;

    std::map<std::string,Tlocation> locations;          // Locations in the environment (label, Tlocation)
    std::map<std::string,Tpassage> passages;            // Passages in the environment (label, Tpassage)

    json topo_map_json;
    std::string user_id;                                // the ID (unique identifier) of the user
    std::string user_name, user_surname;                // the name of the user to interact with
    std::string user_language;                          // The language of the user
    std::string user_location;                          // most probable user location (from VC)
    std::string robot_location;                         // most probable robot location (from task manager under PATROL task)
    std::map<int,std::string> tasks_in_execution;       // List of tasks in execution (waiting for response)    
    std::map<std::string, Ttask_statistics> tasks_statistics;       // List of tasks and their historial statistics
    std::string task_statistics_file_path;
    bool alert_active;                                  // To avoid multiple calls of the Alerts
    bool teleop_active;                                 // To vaoid multiple teleoperation calls
    bool os_active;
    int os_task_id;
    bool use_hgw;                                       // Set if there is a Home GateWay server
    std::string hgw_env_url, hgw_user_url, local_env_url, local_user_url;   // the url of the HGW or local file to read environment parameters
    bool e_stop_active;                                 // to control when the Emergency stop has been pressed.
    bool ready_to_work;                                 // General boolean to turn on/off the system
    bool offline_warning_active;                        // To discern when the robot is offline
    ros::Time last_keepAlive;

    // Curl
    char errorBuffer[CURL_ERROR_SIZE];
    std::string curl_buffer;
    //bool init_curl(CURL *&conn, std::__cxx11::string url);
    static int writer(char *data, size_t size, size_t nmemb, std::string *writerData);
};

#endif
