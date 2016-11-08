#!/usr/bin/env python
"""
@package contact_detection
@file contact_detection_action_server.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import rospy

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty
from ar_signal_processing import SignalAnalysis
from ar_geometry_msg_conversion import wrench_to_array
import actionlib
import contact_detection.msg

class WrenchContactDetectorNode(object):

    def __init__(self, name="wrench_contact_detector"):
        self._name = name
        self._is_init_ok = False
        self._analysis = SignalAnalysis(size=6,
                                        num_sample_init=500,
                                        deviation_max=30)
        self._is_init_ok = True

        # ###########################
        # initialising ros interface
        # for receiving the wrenches (only launched at action execution)
        self._sub_wrench = None
        # for storing the action server
        self._action_server = actionlib.SimpleActionServer(self._name,
                                                           contact_detection.msg.DetectContactAction,
                                                           execute_cb=self._execute_cb,
                                                           auto_start=False)
        # for enabling sensor reset (always launched).
        self._sub_reset = rospy.Subscriber("wrench_reset", Empty, self._reset_callback)
        self._action_server.start()

    def _reset_callback(self):
        """
        when a wrench reset is asked, purges the signal analysis component
        """
        self._analysis.clear_std()

    def _execute_cb(self, goal):
        """
        adress an action request
        :param goal the provided spec of the current action
        """
        rospy.loginfo("{} launching contact detector action with spec \n {}".format(self._name, goal))
        feedback = contact_detection.msg.DetectContactFeedback()
        result = contact_detection.msg.DetectContactResult()

        rate = rospy.Rate(goal.frequency)

        if goal.do_noise_calibration:
            self._analysis.clear_std()

        sub_wrench = rospy.Subscriber("wrench", WrenchStamped, self._wrench_callback)

        end_loop = False
        feedback.is_in_contact = False

        while not end_loop:

            feedback.is_in_contact = self._analysis._std_violation

            if not self._action_server.is_preempt_requested():
                self._action_server.publish_feedback(feedback)

            if goal.finish_on_contact and feedback.is_in_contact:
                end_loop = True
            else:
                end_loop = self._action_server.is_preempt_requested() or rospy.is_shutdown()

            if not end_loop:
                rate.sleep()

        # we unregister from the topic
        sub_wrench.unregister()

        if self._action_server.is_preempt_requested():
            self._action_server.set_preempted()
        else:
            result.is_in_contact = feedback.is_in_contact

            if result.is_in_contact:
                self._action_server.set_succeeded(result)
            else:
                self._action_server.set_aborted(result)
        rospy.loginfo("End of the action")


    def _wrench_callback(self, msg):
        # rospy.loginfo("{} received: {}".format(name, msg))
        self._analysis.process(wrench_to_array(msg.wrench))
        # print "output is: {}".format(output)

if __name__ == '__main__':
    name = "wrench_contact_detector"
    rospy.init_node(name, anonymous=True)

    detector = WrenchContactDetectorNode(name)

    if detector._is_init_ok:
        rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")
