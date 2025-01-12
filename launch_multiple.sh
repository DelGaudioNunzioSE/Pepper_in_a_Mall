#!/bin/bash


#chmod +x launch_multiple.sh
#./launch_multiple.sh


roslaunch ros_audio_pkg speech_recognition.launch &


#roslaunch your_package second_launch_file.launch &


wait # Wait for all background processes to finish
