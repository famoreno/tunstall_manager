#!/usr/bin/env python
from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *
import tkinter as Tkinter
import tkinter.font as tkFont
import string
import os
import json


class bt_gui():

    # VARS of the class
    bt_name = None          # Name of the bt_root
    tk_root = None          # A TKinter window root    
    tk_T = None             # Text widget (where we will write the Tree structure)
    it = 0                  # Iteration counter
    task_active_found = False

    
    def __init__(self, name):
        self.tk_root = Tkinter.Tk(className=name)                       # Creates a TK inter window root
        self.bt_name = name                                             # Name of the BT root
        self.dFont = tkFont.Font(family="Calibri", size=6)
        self.tk_T = Tkinter.Text(self.tk_root, height=50, width=50, font=self.dFont)     # Text widget: TK.Text( master, option, ... )

        self.tk_T.pack(side=Tkinter.LEFT, fill=Tkinter.BOTH, expand=Tkinter.YES)
        yscrollbar = Tkinter.Scrollbar(self.tk_root, orient=Tkinter.VERTICAL, command=self.tk_T.yview)
        yscrollbar.pack(side=Tkinter.RIGHT, fill=Tkinter.Y)
        self.tk_T["yscrollcommand"] = yscrollbar.set

        self.config_tags()
        self.tk_T.insert(Tkinter.END, self.bt_name + "-it: " + str(self.it))    # TK.Text.insert(index [,string]...)
        self.tk_T.pack()                                                # organizes widgets in blocks


    def shutdown(self):
        print("[pi_trees_gui] Closing...")
        self.tk_root.quit()
        self.tk_root.destroy()


    def config_tags(self):
        self.tk_T.tag_config("running", foreground="orange")
        self.tk_T.tag_config("success", foreground="green")
        self.tk_T.tag_config("failure", foreground="red")

        big_font = tkFont.Font(weight="bold", size=9)
        self.tk_T.tag_config("big", font=big_font)

        bold_font = tkFont.Font(weight="bold",size=6)
        self.tk_T.tag_config("bold", font=bold_font)


    def remove_tags(self, start, end):
        for tag in self.tags.keys():
            self.tk_T.tag_remove(tag, start, end)


    #def start(self):
    #    self.my_print_tree(self.bt_root, indent=0, use_symbols=True)
    #    self.tk_root.update_idletasks()
    #    self.tk_root.update()                                           # Update TKinter visualization


    def updateGui(self, tree, use_colors=True):
        self.it = self.it+1                                             # Increase it counter
        self.tk_T.delete('1.0', Tkinter.END)                            # Delete current text (= erese all)
        self.tk_T.insert(Tkinter.END, self.bt_name + "-it: " + str(self.it)+"\n")    # Update text

        self.task_active_found = False
        self.print_tree(tree, 0, use_colors)                            # Print tree on the TK.Text widget
        self.tk_root.update_idletasks()
        self.tk_root.update()                                           # Update TKinter visualization
    
    
    def print_tree(self, tree, indent=0, use_colors=False):
        """
            Print an ASCII representation of the bt tree in the tk inter window
            Its a recursive function!
        """
        if use_colors:
            if indent == 0:
                self.print_tree_symbol(tree, indent)
                indent += 1
            for c in tree.children:
                self.print_tree_symbol(c, indent)
                try:
                    if c.children != []:
                        self.print_tree(c, indent+1, use_colors)
                except:
                    pass

        else:
            for c in tree.children:
                strOne = "    " * indent                            # get index according to level on the tree (indent)
                strTwo = "-->" + c.name                             # Name of the children
                self.tk_T.insert(Tkinter.END, strOne+strTwo+"\n")   # Write children
                try:
                    if c.children != []:
                        self.print_tree(c, indent+1, use_colors)       # Recursive
                except:
                    pass
                    
    
    def print_tree_symbol(self, c, indent):
        """
            Use ASCII symbols to represent Sequence, Selector, Task, etc.
            Also attach the Task_Status (running, success, failure)
        """
        # 1. Add task symbol
        strOne = "    " * indent        # Set index tab
        if isinstance(c, Selector):
            strTwo = "--?"
        elif isinstance(c, Sequence) or isinstance(c, Iterator):
            strTwo = "-->"
        elif isinstance(c, RandomSequence) or isinstance(c, RandomIterator):
            strTwo = "~~>"
        elif isinstance(c, RandomSelector):
            strTwo = "~~?"
        elif isinstance(c, Loop):
            strTwo = "<->"
        elif isinstance(c, Invert):
            strTwo = "--!"
        else:
            strTwo = "--|"

        # 2. Only for first lvl tasks: add Task(id) assigned by the bt_manager
        strThird = " "
        if indent == 1:
            strThird = " [" + str(c.id)
            # If permanent task
            if c.permanence:
                strThird += "-P] "
            else:
                strThird += "] "

        self.tk_T.insert(Tkinter.END, strOne + strTwo + strThird)

        # 3. Add Task Name with color according to status.
        strTaskName = c.name + " "
        len_TaskName = len(strTaskName)+1
        #strTaskStatus = self.returnNodeStatusAsString(c.status)
        #self.tk_T.insert(Tkinter.END, strTaskName+strTaskStatus)
        self.tk_T.insert(Tkinter.END, strTaskName)

        # Color the status (if any)
        #--------------------------
        if (c.status == TaskStatus.FAILURE):
            pos1 = self.tk_T.index("end-%dc"%len_TaskName)      # (row,col) of start of current line
            pos2 = self.tk_T.index("end-1c")                    # (row,col) of end of current line
            self.tk_T.tag_add('failure', pos1, pos2)            # index are (row, col)
        elif (c.status == TaskStatus.SUCCESS):
            pos1 = self.tk_T.index("end-%dc"%len_TaskName)      # (row,col) of start of current line
            pos2 = self.tk_T.index("end-1c")                    # (row,col) of end of current line
            self.tk_T.tag_add('success', pos1, pos2)            # index are (row, col)
        elif (c.status == TaskStatus.RUNNING):
            pos1 = self.tk_T.index("end-%dc"%len_TaskName)      # (row,col) of start of current line
            pos2 = self.tk_T.index("end-1c")                    # (row,col) of end of current line
            self.tk_T.tag_add('running', pos1, pos2)            # index are (row, col)
            if self.task_active_found == False and ( isinstance(c, SimpleActionTask) or isinstance(c, GenericTask) or isinstance(c, WaitSec) ):
                self.tk_T.tag_add('bold', pos1, pos2)           # index are (row, col)
                self.task_active_found = True

        # 4. Add Execution time, and trace
        self.tk_T.insert(Tkinter.END, " (" + str(c.execution_time) + ")s")

        # Force new line
        self.tk_T.insert(Tkinter.END, "\n" )


    def returnNodeStatusAsString(self, nodeStatus):
        """
            returns a String containing the status of the node based on the
            possible values of the enum type TaskStatus from pi_trees_lib
        """
        if (nodeStatus == TaskStatus.FAILURE):
            value = "FAILURE"
        elif (nodeStatus == TaskStatus.RUNNING):
            value = "RUNNING"
        elif (nodeStatus == TaskStatus.SUCCESS):
            value = "SUCCESS"
        else:
            value = ""
        return value


    def print_phpsyntax_tree(self, tree):    
        """
            Print an output compatible with ironcreek.net/phpSyntaxTree
        """
        for c in tree.children:
            print ("[" + string.replace(c.name, "_", ".")),
            if c.children != []:
                print_phpsyntax_tree(c),
            print ("]"),
        
    
    def print_dot_tree(root, dotfilepath=None):
        """
            Print an output compatible with the DOT synatax and Graphiz
        """
        gr = pgv.AGraph(strict=True, directed=True, rotate='0', bgcolor='white', ordering="out")
        gr.node_attr['fontsize'] = '9'
        gr.node_attr['color'] = 'black'
        
        if dotfilepath is None:
            dotfilepath = os.path.expanduser('~') + '/.ros/tree.dot'
        
        global last_dot_tree
        
        # Add the root node
        gr.add_node(root.name)
        node = gr.get_node(root.name)
        if root.status == TaskStatus.RUNNING:
            node.attr['fillcolor'] = 'yellow'
            node.attr['style'] = 'filled'
            node.attr['border'] = 'bold'
        elif root.status == TaskStatus.SUCCESS:
            node.attr['color'] = 'green'
        elif root.status == TaskStatus.FAILURE:
            node.attr['color'] = 'red'
        else:
            node.attr['color'] = 'black'
            

    def updateGuiJson(self, json_tree):
        # Delete current text (= erese all)
        self.tk_T.delete('1.0', Tkinter.END)
        # Increase iteration counter
        self.it = self.it+1
        self.tk_T.insert(Tkinter.END, self.bt_name + "-it: " + str(self.it)+"\n")    # Update text

        # Remove "\" characters added when being tranfered as a ROS topic
        json_tree = str(json_tree)
        json_tree = ''.join(ch for ch in json_tree if ch!="\\")
        # Ensure fields are separated with " not with '
        json_tree = json_tree.replace("'", "\"")

        # json
        py_tree = json.loads(json_tree)                 # Parse as a Python dictionary

        self.task_active_found = False
        self.print_tree_from_json(py_tree, indent=0)    # Print tree on the TK.Text widget
        self.tk_root.update_idletasks()
        self.tk_root.update()                           # Update TKinter visualization



    def print_tree_from_json(self, py_tree, indent=0):
        """
            Print an ASCII representation of the bt tree in the tk inter window
            Its a recursive function!
        """
        if indent == 0: # ROOT of the tree
            self.print_tree_symbol_json(py_tree['Node'], indent)
            indent += 1

        for c in py_tree['Node']['children']:
            self.print_tree_symbol_json(c['Node'], indent)
            try:
                if c['Node']['children'] != []:
                    self.print_tree_from_json(c, indent+1)
            except:
                pass

                

    def print_tree_symbol_json(self, py_ch, indent):
        """
            Use ASCII symbols to represent Sequence, Selector, Task, etc.
            Colors according to the Task_Status (running, success, failure)
            Include the Task Priority

        """
        # Force new line
        if indent == 1:
            self.tk_T.insert(Tkinter.END, "\n" )

        # 1. Add task symbol
        strOne = "    " * indent        # Set index tab
        if py_ch['type'] == "Selector":
            strTwo = "--?"
        elif py_ch['type'] == "Sequence" or py_ch['type'] == "Iterator":
            strTwo = "-->"
        elif py_ch['type'] == "RandomSequence" or py_ch['type'] == "RandomIterator":
            strTwo = "~~>"
        elif py_ch['type'] == "RandomSelector":
            strTwo = "~~?"
        elif py_ch['type'] == "Loop":
            strTwo = "<->"
        elif py_ch['type'] == "Invert":
            strTwo = "--!"
        else:
            strTwo = "--|"

        # 2. Only for first lvl tasks: add Task(Priority)
        strThird = " "
        if indent == 1:
            strThird = " [" + py_ch['priority']
            # If permanent task
            if py_ch['permanence'] in ["True", "true", "1"]:
                strThird += "-P] "
            else:
                strThird += "] "

        self.tk_T.insert(Tkinter.END, strOne + strTwo + strThird)

        # 3. Add Task Name with color according to status.
        strTaskName = py_ch['name'] + " "
        len_TaskName = len(strTaskName)+1
        self.tk_T.insert(Tkinter.END, strTaskName)

        # Color the status (if any)
        #--------------------------
        if indent <= 1:
            pos1 = self.tk_T.index("end-%dc"%len_TaskName)      # (row,col) of start of current line
            pos2 = self.tk_T.index("end-1c")                    # (row,col) of end of current line
            self.tk_T.tag_add('bold', pos1, pos2)               # index are (row, col)

        if (py_ch['status'] == "Failure"):
            pos1 = self.tk_T.index("end-%dc"%len_TaskName)      # (row,col) of start of current line
            pos2 = self.tk_T.index("end-1c")                    # (row,col) of end of current line
            self.tk_T.tag_add('failure', pos1, pos2)            # index are (row, col)
        elif (py_ch['status'] == "Success"):
            pos1 = self.tk_T.index("end-%dc"%len_TaskName)      # (row,col) of start of current line
            pos2 = self.tk_T.index("end-1c")                    # (row,col) of end of current line
            self.tk_T.tag_add('success', pos1, pos2)            # index are (row, col)
        elif (py_ch['status'] == "Running"):
            pos1 = self.tk_T.index("end-%dc"%len_TaskName)      # (row,col) of start of current line
            pos2 = self.tk_T.index("end-1c")                    # (row,col) of end of current line
            self.tk_T.tag_add('running', pos1, pos2)            # index are (row, col)
            if self.task_active_found == False and indent == 1: # and ( isinstance(c, SimpleActionTask) or isinstance(c, GenericTask) or isinstance(c, WaitSec) ):
                self.tk_T.tag_add('big', pos1, pos2)           # index are (row, col)
                self.task_active_found = True
        #else:
            # Taks is not yet executted.... do nothing!



        # 4. Add Execution time, and trace
        self.tk_T.insert(Tkinter.END, " (" + py_ch['runtime'] + ")s")

        # Force new line
        self.tk_T.insert(Tkinter.END, "\n" )
