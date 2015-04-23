#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import vigir_footstep_planning_msgs.msg

from vigir_footstep_planning_msgs.msg import ErrorStatus

# creates error status with given error and/or warning code and message
def create_error_status(self, error = 0, error_msg = "", warning = 0, warning_msg = ""):
    error_status = ErrorStatus()

    if (error != 0):
        error_status.error = error
        error_status.error_msg = "[Error] " + error_msg

    if (warning != 0):
        warning_status.warning = warning
        warning_status.warning_msg = "[Warning] " + warning_msg        

    return error_status

