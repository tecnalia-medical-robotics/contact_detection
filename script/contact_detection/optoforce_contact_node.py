#!/usr/bin/env python
"""
@package contact_detection
@file ar_signal_processing.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import rospy

from geometry_msgs.msg import Point, WrenchStamped
from ar_signal_processing import SignalAnalysis
from ar_geometry_msg_conversion import wrench_to_array

class WrenchContactDetectorNode(object):

    def __init__(self, name="WrenchContactDetector"):
        self._name = name
        self._is_init_ok = False
        self._analysis = SignalAnalysis(size=6, num_sample_init=50)

        self._is_init_ok = True

        # ###########################
        # initialising ros interface
        # for receiving the wrenches
        self.sub_wrench =  rospy.Subscriber("wrench", WrenchStamped, self._wrench_callback)


    def _wrench_callback(self, msg):
        # rospy.loginfo("{} received: {}".format(name, msg))

        output = self._analysis.process(wrench_to_array(msg.wrench))
        print "output is: {}".format(output)


if __name__ == '__main__':
    name = "wrench_contact_detector"
    rospy.init_node(name, anonymous=True)

    detector = WrenchContactDetectorNode(name)


    if detector._is_init_ok:
        rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")
