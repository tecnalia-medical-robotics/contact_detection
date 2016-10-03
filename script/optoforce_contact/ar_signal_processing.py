#!/usr/bin/env python
"""
@package optoforce_contact
@file ar_signal_processing.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import numpy

class SignalAnalysis(object):
    """

    """
    def __init__(self, size=1, num_sample_init = 50):

        self._num_sample_init = num_sample_init
        self._size = size
        self._signals = []
        for i in xrange(size):
            self._signals.append([])
        self._mean = None
        self._std = None
        self._num_sample = 0
        self._deviation_max = 10

    def process(self, data):
        data_array = numpy.asarray(data)
        size, = data_array.shape
        if size != self._size:
            print "data shape: {} not consistent with spec {}".format(size, self._size)
            return False

        self._num_sample +=1

        if self._num_sample < self._num_sample_init:
            for i, val in enumerate(data_array):
                self._signals[i].append(val)
            return False

        if self._num_sample == self._num_sample_init:
            # we compute the mean and std
            self._mean = numpy.mean(self._signals)
            self._std = numpy.std(self._signals, ddof=1)
            return False

        if self._num_sample > self._num_sample_init:
            deviation = numpy.abs(data_array - self._mean)

            # print "deviation obtained : {}".format(deviation)
            max_violation = deviation > (self._deviation_max * self._std)

            return numpy.any(max_violation)
