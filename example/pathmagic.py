#!/usr/bin/env python3
import os
import sys


class context:
    def __enter__(self):
        # assume the working directory is always the main directory
        sys.path.append(os.getcwd() + '/externals/CBBA-Python/lib')
        sys.path.append(os.getcwd() + '/build')
        sys.path.append(os.getcwd() + '/src')

    def __exit__(self, *args):
        pass
