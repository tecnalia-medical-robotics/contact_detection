#!/usr/bin/env python
"""
@package contact_detection
@file ar_geometry_msgs_conversion.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
"""


import numpy


def wrench_to_array(wrench):
    """
    convert a geometry_msgs array into an 1d array
    :param wrench: the wrench to convert
    :return: the related array
    """
    return numpy.array([wrench.force.x, wrench.force.y, wrench.force.z,
                         wrench.torque.x, wrench.torque.y, wrench.torque.z])
