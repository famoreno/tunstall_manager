#!/usr/bin/env python

import rospy
from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *
from pi_trees_gui.pi_trees_gui import *
import numpy as np


class bt_viewer:

    # VARS of the class
    bt_json = None              # Root of the BT
    bt_manager_gui = None       # Handler of the GUI for the BT

    # =============================================================
    # ==========================  CALLBACK  =======================
    # =============================================================
    def callbackTree(self, json_tree):
        self.bt_json = json_tree.data;
        # We cannot update there the GUI because this is a thread


    # ---------------------------------------------------------------------
    #                           INIT (MAIN)
    # ---------------------------------------------------------------------
    def __init__(self):
        rospy.init_node("bt_viewer")
        self.verbose = rospy.get_param("~verbose",False)

        # Create the GUI
        # ------------------
        self.bt_manager_gui = bt_gui("BT-GUI")      # Creates a GUI with a Text widget
        # Set the shutdown function
        rospy.on_shutdown(self.shutdown)


        # TOPIC SUBSCRIBER
        # -----------------
        rospy.Subscriber('/bt_manager/tree', String, self.callbackTree) # python creates a different thread for each callback (carefull with the GUI)

        # LOOP
        while not rospy.is_shutdown():
            # spin_once() --> in python there is no need to spin_once (they are different threads)
            if self.bt_json is not None:
                self.bt_manager_gui.updateGuiJson(self.bt_json)
            rospy.sleep(0.1)  # SEC    This set the freq at with the tree is updated


    # =============================================================
    # ==========================  SHUTDOWN  =======================
    # =============================================================
    def shutdown(self):
        rospy.loginfo("[bt_viewer] Closing GUI...")
        self.bt_manager_gui.shutdown()
        rospy.sleep(0.1)





    # =============================================================
    # ==========================  SHUTDOWN  =======================
    # =============================================================
    def publish_tree(self, tree):
        """
            A serialization of the task tree in JSON format.
        """
        json_tree = '{' + self.print_tree(tree) + '}'
        self.tree_pub.publish(json_tree)
        print (json_tree)


    def print_tree(self, tree):
        """
            Print an ASCII representation of the bt tree in JSON format
            Its a recursive function!
        """
        json_node = '"' + str(tree.name) + '":{'
        json_node += ' "id":"' + str(tree.id) + '",'
        json_node += ' "type":"' + self.get_node_type(tree) + '",'
        json_node += ' "status":"' + self.get_node_status(tree) + '",'
        json_node += ' "children":{'

        if tree.children != []:
            for c in tree.children:
                try:
                    json_ch = self.print_tree(c)       # Recursive call
                    json_node += json_ch + ','
                except:
                    pass
            json_node = json_node[:-1]  # remove last char ","

        json_node += '}'    # end of children
        json_node += '}'    # end of this node
        return json_node



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

    def get_node_status(self, node):
        if node.status is 0:
            return "Failure"
        elif node.status is 1:
            return "Success"
        elif node.status is 2:
            return "Running"
        else:
            return "Unknown"
# =============================================================
# ==========================  MAIN  ===========================
# =============================================================

if __name__ == '__main__':
    tree = bt_viewer()
