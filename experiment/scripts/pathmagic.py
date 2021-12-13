#!/usr/bin/env python3
import os
import sys


class context:
    def __init__(self, EXTERNAL_FLAG=False):
        # True if use external libraries
        self.external_flag = EXTERNAL_FLAG

    def __enter__(self):
        # assume the working directory is always the main directory
        sys.path.append(os.getcwd() + '/build')
        sys.path.append(os.getcwd() + '/src')
        sys.path.append(os.getcwd() + '/experiment/lib')
        
        # True if use external libraries
        if self.external_flag:
            sys.path.append(os.path.expanduser("~") + '/Mambo-Tracking-Interface/lib')

    def __exit__(self, *args):
        pass
