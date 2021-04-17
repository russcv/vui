#!/usr/bin/env python

import rospy
from vui.voice import import_voice as vi

def print_hello(a):
    print(a)


if __name__ == "__main__":
    a = vi()
    print_hello(a)
