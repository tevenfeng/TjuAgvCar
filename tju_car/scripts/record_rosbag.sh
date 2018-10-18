#!/bin/bash

cd /home/nvidia/AutonomousTju

rosbag record /joy /scan -O baggy.bag
