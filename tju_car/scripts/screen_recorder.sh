#!/bin/bash

gst-launch-1.0 -v ximagesrc use-damage=0 ! nvvidconv ! 'video/x-raw(memory:NVMM),alignment=(string)au,format=(string)I420,framerate=(fraction)25/1,pixel-aspect-ratio=(fraction)1/1' ! omxh264enc !  'video/x-h264,stream-format=(string)byte-stream' ! h264parse ! matroskamux ! filesink location=/home/nvidia/AutonomousTju/data/screen.mkv
