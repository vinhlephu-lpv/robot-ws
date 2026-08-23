#!/usr/bin/env python3
"""
Convenience launcher for controller response plotting and MATLAB-style tuning.
"""
import sys
import os

pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

from my_robot_controller.plot_response import main

if __name__ == '__main__':
    main()
