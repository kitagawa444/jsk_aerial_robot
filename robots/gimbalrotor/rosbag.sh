#!/bin/bash

rosbag record -a -x "(.*)livox/lidar(.*)|(.*)livox/lidar_orig(.*)|(.*)cloud_registered_body(.*)|" #(.*)cloud_registered_body(.*)
