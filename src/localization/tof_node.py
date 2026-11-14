#!/usr/bin/env python
"""Run the installed VL53L1X ROS driver"""

import os
import sys


def main():
    os.execvp('rosrun', ['rosrun', 'vl53l1x', 'vl53l1x_node'] + sys.argv[1:])


if __name__ == '__main__':
    main()
