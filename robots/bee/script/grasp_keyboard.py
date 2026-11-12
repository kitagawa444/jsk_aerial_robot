#!/usr/bin/env python3
"""Send mission requests to the central controller; no flight publishers."""
import argparse
import os
import select
import sys
import termios
import tty

import rospy
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/bee_cooperative_grasp')
    args = parser.parse_args(rospy.myargv()[1:])
    if not sys.stdin.isatty():
        parser.error('an interactive terminal is required')
    rospy.init_node('grasp_keyboard', anonymous=True)
    root = args.namespace.rstrip('/')
    services = {key: rospy.ServiceProxy(root + '/' + name, Trigger)
                for key, name in [('g', 'start'), ('t', 'transport'), ('x', 'abort')]}
    print('g: start  t: transport from Hold  x/Ctrl-C: abort and exit')
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        # Read Ctrl-C as a key so it sends the abort service before exiting.
        key_settings = termios.tcgetattr(sys.stdin)
        key_settings[3] &= ~termios.ISIG
        termios.tcsetattr(sys.stdin, termios.TCSANOW, key_settings)
        while not rospy.is_shutdown():
            if not select.select([sys.stdin], [], [], .1)[0]:
                continue
            key = os.read(sys.stdin.fileno(), 1).decode('utf-8', 'ignore')
            if key == '\x03':
                key = 'x'
            if key not in services:
                continue
            try:
                response = services[key]()
                print(('OK: ' if response.success else 'Rejected: ') + response.message)
            except rospy.ServiceException as error:
                print(str(error))
            if key == 'x':
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
