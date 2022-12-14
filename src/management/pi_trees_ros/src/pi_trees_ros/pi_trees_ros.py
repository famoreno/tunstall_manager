#!/usr/bin/env python

"""
    pi_trees_ros.py - Version 0.1 2013-08-28
    
    ROS wrappers for the pi_trees_lib.py library
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from pi_trees_lib.pi_trees_lib import *
import sys


class GenericTask(Task):
    """
        A generic Task that simply calls a callback function with the given parameters.
    """
    def __init__(self, name, cb, params=None, reset_after=False, announce=False):
        super(GenericTask, self).__init__(name)
        self.cb = cb
        self.params = params
        self.reset_after = reset_after
        self.announce = announce
        self.task_started = False
        self.task_finished = False
        self.activate_time = None
        #rospy.loginfo("GenericTask-" + name + "] Creating a new GenericTask")

    def run(self):
        if not self.task_started:
            self.task_started = True
            self.activate_time = rospy.Time.now()

        if not self.task_finished:
            task_status = self.cb(self.params)  # Fire the callback and get status
            if task_status != TaskStatus.RUNNING:
                self.task_finished = True
                self.execution_time = (rospy.Time.now() - self.activate_time).to_sec()
                self.status = task_status
                # Announce the result
                if self.announce:
                    self.announce_result()
                # Reset the task if the reset_after flag is True
                if self.reset_after:
                    self.reset()
            return task_status
        else:
            return self.status

    def reset(self):
        super(GenericTask, self).reset()
        self.task_started = False
        self.task_finished = False
        self.activate_time = None


class MonitorTask(Task):
    """
        Turn a ROS subscriber into a Task.        
        The cb_function (executted when the topic is updated) is responsible of returning the STATUS
        On every "run" it returns the result of the last cb_call
    """
    def __init__(self, name, topic, msg_type, msg_cb, wait_for_message=True, timeout=5, announce=False):
        super(MonitorTask, self).__init__(name)
        
        self.topic = topic
        self.msg_type = msg_type
        self.timeout = timeout
        self.msg_cb = msg_cb
                
        #rospy.loginfo("[MonitorTask-" + topic + "] Subscribing to topic: " + topic)
        
        if wait_for_message:
            try:
                rospy.wait_for_message(topic, msg_type, timeout=self.timeout)
                #rospy.loginfo("[MonitorTask-" + topic + "] Connected.")
            except:
                rospy.logerr("[MonitorTask-" + topic + "] Timed out waiting for msg: " + topic)
                
        # Subscribe to the given topic with the given callback function executed via run() 
        rospy.Subscriber(self.topic, self.msg_type, self._msg_cb)
        
    def _msg_cb(self, msg):
        self.set_status(self.msg_cb(msg))
        
    def run(self):
        return self.status


class ServiceTask(Task):
    """
        Turn a ROS service into a Task.
        It always returns SUCCESS if service is available, and FAILURE otherwise.
        It implements memory
    """
    def __init__(self, name, service, service_type, request,
                 result_cb=None, wait_for_service=True, timeout=5, reset_after=False, announce=False):
        super(ServiceTask, self).__init__(name)
        
        self.result = None
        self.request = request
        self.timeout = timeout
        self.result_cb = result_cb
        self.reset_after = reset_after
        self.announce = announce
        self.service_called = False
        self.activate_time = None
                
        #rospy.loginfo("[ServiceTask-" + service + "] Connecting to service " + service)
        
        if wait_for_service:
            #rospy.loginfo("[ServiceTask-" + service + "] Waiting for service")
            rospy.wait_for_service(service, timeout=self.timeout)
            #rospy.loginfo("[ServiceTask-" + service + "] Connected.")
        
        # Create a service proxy
        self.service_proxy = rospy.ServiceProxy(service, service_type)
        
    def run(self):
        if not self.service_called:
            self.service_called = True
            try:
                self.activate_time = rospy.Time.now()
                result = self.service_proxy(self.request)
                if self.result_cb is not None:
                    self.result_cb(result)
                self.execution_time = (rospy.Time.now() - self.activate_time).to_sec()
                self.status = TaskStatus.SUCCESS
                # Announce the result
                if self.announce:
                    self.announce_result()
                # Reset the task if the reset_after flag is True
                if self.reset_after:
                    self.reset()
                return self.status
            except:
                rospy.logerr(sys.exc_info())
                self.status = TaskStatus.FAILURE
                # Announce the result
                if self.announce:
                    self.announce_result()
                return self.status
        else:
            return self.status

    def reset(self):
        self.status = None
        self.service_called = False
        self.activate_time = None
        super(ServiceTask, self).reset()


class ServiceTaskDynamic(Task):
    """
        Turn a ROS service into a Task.
        It always returns SUCCESS if service is available, and FAILURE otherwise.
        It implements memory, and the request value can be updated before running the service call
    """
    def __init__(self, name, service, service_type, request_cb,
                 result_cb=None, wait_for_service=True, timeout=5, reset_after=False, announce=False):
        super(ServiceTaskDynamic, self).__init__(name)
        
        self.result = None
        self.request_cb = request_cb
        self.result_cb = result_cb
        self.timeout = timeout
        self.reset_after = reset_after
        self.announce = announce
        self.service_called = False
        self.activate_time = None
                
        #rospy.loginfo("[ServiceTaskDynamic-" + service + "] Connecting to service " + service)
        
        if wait_for_service:
            #rospy.loginfo("[ServiceTask-" + service + "] Waiting for service")
            rospy.wait_for_service(service, timeout=self.timeout)
            #rospy.loginfo("[ServiceTask-" + service + "] Connected.")
        
        # Create a service proxy
        self.service_proxy = rospy.ServiceProxy(service, service_type)
        
    def run(self):
        if not self.service_called:
            self.service_called = True
            try:
                self.activate_time = rospy.Time.now()
                # Update request value
                self.request = self.request_cb()
                # run the srv
                result = self.service_proxy(self.request)
                if self.result_cb is not None:
                    self.result_cb(result)
                self.execution_time = (rospy.Time.now() - self.activate_time).to_sec()
                self.status = TaskStatus.SUCCESS
                # Announce the result
                if self.announce:
                    self.announce_result()
                # Reset the task if the reset_after flag is True
                if self.reset_after:
                    self.reset()
                return self.status
            except:
                rospy.logerr(sys.exc_info())
                self.status = TaskStatus.FAILURE
                # Announce the result
                if self.announce:
                    self.announce_result()
                return self.status
        else:
            return self.status

    def reset(self):
        self.status = None
        self.service_called = False
        self.activate_time = None
        super(ServiceTaskDynamic, self).reset()



class SimpleActionTask(Task):
    """
        Turn a ROS action into a Task.
        It has "memory", it only calls the actionServer after "reset". By default: reset_after=False
    """
    def __init__(self, name, action, action_type, goal, rate=5, connect_timeout=10, result_timeout=30,
                 reset_after=False, active_cb=None, done_cb=None, feedback_cb=None, announce=False):
        super(SimpleActionTask, self).__init__(name)

        self.action = action
        self.goal = goal
        self.tick = 1.0 / rate
        self.rate = rospy.Rate(rate)
        self.result = None
        self.connect_timeout = connect_timeout
        self.result_timeout = result_timeout
        self.reset_after = reset_after
        self.announce = announce
        self.final_status = None
        
        if done_cb:
            self.user_done_cb = done_cb
        else:
            self.user_done_cb = None
        
        self.done_cb = self.default_done_cb
        
        if active_cb is None:
            active_cb = self.default_active_cb
        self.active_cb = active_cb
        
        if feedback_cb is None:
            feedback_cb = self.default_feedback_cb
        self.feedback_cb = feedback_cb
                
        self.action_started = False
        self.action_finished = False
        self.goal_status_reported = False
        self.time_so_far = 0.0
        
        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                            'SUCCEEDED', 'ABORTED', 'REJECTED',
                            'PREEMPTING', 'RECALLING', 'RECALLED',
                            'LOST']
        
        self.retry_goal_states = [GoalStatus.PREEMPTED]
            
        #rospy.loginfo("Connecting to action " + action)

        # Subscribe to the base action server
        self.action_client = actionlib.SimpleActionClient(action, action_type)

        #rospy.loginfo("Waiting for action server...")
        
        # Wait up to timeout seconds for the action server to become available
        try:
            self.action_client.wait_for_server(rospy.Duration(self.connect_timeout))
        except:
            rospy.logwarn("[SimpleActionTask] Timed out connecting to the action server " + action)
    
        #rospy.loginfo("Connected to action server")

    def run(self):
        # Send the goal
        if not self.action_started:
            #rospy.loginfo("Sending " + str(self.name) + " goal to action server...")
            self.action_client.send_goal(self.goal, done_cb=self.done_cb,
                                         active_cb=self.active_cb, feedback_cb=self.feedback_cb)
            self.action_started = True
            self.activate_time = rospy.Time.now()
                
        if not self.action_finished:
            self.time_so_far += self.tick
            self.rate.sleep()
            if self.time_so_far > self.result_timeout:
                self.action_client.cancel_goal()
                #rospy.loginfo("[SimpleActionTask] Timed out achieving goal. CANCELING current Goal.")
                self.action_finished = True
                self.status = TaskStatus.FAILURE
                self.goal_status = self.status
                # Announce the result
                self._duration = rospy.Time.now() - self.activate_time
                self.execution_time = self._duration.to_sec()
                self.trace =  '"' + self.name +  '" : "Timed out achieving goal"'
                if self.announce:
                    self.announce_result()
                return TaskStatus.FAILURE
            else:
                return TaskStatus.RUNNING
        else:
            # this secction is executed after finishing and until reseted
            # Check the final goal status returned by default_done_cb
            if self.goal_status == GoalStatus.SUCCEEDED:
                self.status = TaskStatus.SUCCESS
            
            # This case handles PREEMPTED BY TIMEOUT
            elif self.time_so_far > self.result_timeout:
                self.status = TaskStatus.FAILURE

            # This case handles PREEMPTED but RETRY
            elif self.goal_status in self.retry_goal_states:
                self.status = TaskStatus.RUNNING
                self.action_started = False
                self.action_finished = False
                self.time_so_far = 0
            
            # Otherwise, consider the task to have failed
            else:
                self.status = TaskStatus.FAILURE
            
            # Store the final status before we reset
            self.final_status = self.status

            if not self.goal_status_reported:
                self.goal_status_reported = True

                # Announce the result
                self._duration = rospy.Time.now() - self.activate_time
                self.execution_time = self._duration.to_sec()
                if self.announce:
                    #rospy.loginfo("[pi_tree_actionlib] ANNOUNCE ")
                    self.announce_result()

                #rospy.loginfo("[pi_tree_actionlib] Action " + self.name + " terminated after "
                #              + str(self._duration.to_sec()) + " seconds with status "
                #              + self.goal_states[self.action_client.get_state()] + "(" + str(self.status) + ") and resultState = " + str(self.goal_status))


            # Reset the task if the reset_after flag is True
            if self.reset_after:
                self.reset()
        
        ''' We cannot use the wait_for_result() method here as it will block the entire
            tree so we break it down in time slices of duration 1 / rate.
        '''
        self.action_client.wait_for_result(rospy.Duration(10))
        return self.final_status

    def default_done_cb(self, result_state, result):
        """Goal Done Callback
        This callback resets the active flags and reports the duration of the action.
        Also, if the user has defined a result_cb, it is called here before the
        method returns.
        result_state :is the actionlib::SimpleClientGoalStat --> PENDING, ACTIVE ...
        result       :contains the resulting values as defined in the action msg
        """
        # Navigation ended!
        self.goal_status = result_state
        self.action_finished = True

        # Note: Originally this was after the goal_status_reported
        if self.user_done_cb:
            self.user_done_cb(result_state, result)

    
    def default_active_cb(self):
        pass
        
    def default_feedback_cb(self, msg):
        pass
    
    def reset(self):
        # rospy.logerr("[PI_TREE_ROS-ACTIONSERVER : RESETTING all goals in action server " + str(self.name))
        # reset while not finished? cancel_goals to stop ongoing ActionServer
        self.action_client.cancel_all_goals()

        self.action_started = False
        self.action_finished = False
        self.goal_status_reported = False
        # self.status = self.final_status

        self.time_so_far = 0.0
        super(SimpleActionTask, self).reset()


class WaitSec(Task):
    """
        This is a non-blocking wait task, returning RUNNING while waiting.
        The interval argument is in seconds.
    """

    def __init__(self, name, interval, reset_after=False, announce=False):
        super(WaitSec, self).__init__(name)

        self.interval = interval
        self.timer_active = False
        self.reset_after = reset_after
        self.announce = announce

    def run(self):
        if self.timer_active:
            if (rospy.get_rostime() - self.time_start).to_sec() >= self.interval:
                # End of wait interval...
                self.execution_time = (rospy.get_rostime() - self.time_start).to_sec()
                self.status = TaskStatus.SUCCESS
                # Announce the result
                if self.announce:
                    self.announce_result()
                # Reset the task if the reset_after flag is True
                if self.reset_after:
                    self.reset()
                return self.status
            else:
                return TaskStatus.RUNNING

        else:
            #rospy.loginfo("[WaitSec] Waiting for %.2f sec", self.interval)
            self.time_start = rospy.get_rostime()
            self.timer_active = True
            return TaskStatus.RUNNING

    def reset(self):
        self.status = None
        self.timer_active = False
        super(WaitSec, self).reset()
