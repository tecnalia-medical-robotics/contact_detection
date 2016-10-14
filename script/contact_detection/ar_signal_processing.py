#!/usr/bin/env python
"""
@package contact_detection
@file ar_signal_processing.py
@author Anthony Remazeilles
@brief see README.md

Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import numpy
from termcolor import colored
import sys
class SignalAnalysis(object):
    """

    """
    def __init__(self, size=1, num_sample_init = 50, deviation_max=10, verbose=False):

        self._num_sample_init = num_sample_init
        self._size = size
        self._signals = []
        for i in xrange(size):
            self._signals.append([])
        self._mean = None
        self._std = None
        self._num_sample = 0
        self._deviation_max = deviation_max
        self._std_violation = False
        self._verbose = verbose


    def clear_std(self):
        self._signals = []
        for i in xrange(self._size):
            self._signals.append([])
        self._mean = None
        self._std = None
        self._num_sample = 0
        self._std_violation = False
        
    def process(self, data):
        data_array = numpy.asarray(data)
        size, = data_array.shape
        if size != self._size:
            print "data shape: {} not consistent with spec {}".format(size, self._size)
            return False

        self._num_sample +=1
        #print "num_sample = {}".format(self._num_sample)
        if self._num_sample < self._num_sample_init:
            for i, val in enumerate(data_array):
                self._signals[i].append(val)
            self._std_violation = False

        if self._num_sample == self._num_sample_init:
            # we compute the mean and std
            #print self._signals
            self._mean = numpy.mean(self._signals, axis=1)
            self._std = numpy.std(self._signals, axis=1)
            if self._verbose:
                print "[ar_signal_processing] mean : {}".format(self._mean)
                print "[ar_signal_processing] std  : {}".format(self._std)
            
            self._std_violation = False
            
        if self._num_sample > self._num_sample_init:
            deviation = numpy.abs(data_array - self._mean)
            max_violation = deviation > (self._deviation_max * self._std)         
            self._std_violation = numpy.any(max_violation)

        if self._verbose and self._std_violation:
                color = 'red' if self._std_violation else 'blue'
                print "num_sample = {}".format(self._num_sample)
                print colored("dev : {}".format(deviation), color)
                print colored("th  : {}".format(self._deviation_max * self._std), color)
                print colored("viol: {}".format(max_violation), color)
        return self._std_violation 
