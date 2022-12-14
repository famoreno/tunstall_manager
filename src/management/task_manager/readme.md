# Description
This pkg implements a task manager to handle the different tasks our robot must carry out.  
The current implementation is based on Behaviour Trees, using the "pi_trees" library (plus private modifications).
A GUI is provided to easily visualize the current task-plan, as well as to indicate the task being executted.

## Tasks
A task is a definite piece of work assigned to the robot for its execution. 
It can be a simple "SAY_TEXT" or a complex task like "BATTERY_MANAGER" in charge of ensuring the robot battery is always healthy.  

All tasks share a common API with the following fields:  
  * **name**:               The Task's Type. This "name" must correspond with an existing task type in [bt_manager](https://gitlab.com/mapir/mapir-ros-sources/blob/kinetic-dev/planning/task_manager/scripts/bt_manager.py)
  * **priority**:           [0-9]  0=low priority, 9=high priority
  * **permanence**:         False (after 1 execution the task is removed), True (the task is never removed)
  * **args[]**              List of parameters own of each task

## Adding a new Task to the Plan
There are two ways of including a new task to the current plan:
1. Call the **bt_manager/add_new_task** service:  
Calling from Terminal: `rosservice call /bt_manager/add_new_task TaskName TaskPriority TaskPermanence TaskArgs[]`  
e.g. Adding a "say_text" task:    `rosservice call /bt_manager/add_new_task say 8 False ["Buenos dias, Nueva tarea programada con prioridad 8"]`  
e.g. Adding a "go_to_point" task: `rosservice call /bt_manager/add_new_task go_to_point 9 False ["'[3.0,2.0,1.5]'"]`  

2. Set a list of tasks to be executed on start-up
You can see an example [here](https://gitlab.com/mapir/mapir-ros-sources/blob/kinetic-dev/missions_pkg/launch/jgmonroy/jgmonroy_simbot_iro_initial_tasks.yaml)

## Creating a new Task Type
As explained above, the current implementation is based on **Behaviour Trees (BT)**, so every Task must follow the BT convention and be implemented as a Tree.  
In a nutshell, the bt_manager is the ROOT of the tree, and every task to be executed is no more than a branch (a sub-tree). 
The bt_manager is in charge of positioning every task (branch) in the ROOT-tree according to its priority, and removing it if the permanence param is set to False.