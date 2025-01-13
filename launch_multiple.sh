#!/bin/bash


#chmod +x launch_multiple.sh
#./launch_multiple.sh


roslaunch ros_audio_pkg speech_recognition.launch &

#!/bin/bash

# Lista di comandi da eseguire
commands=(
  "roslaunch 'ros_audio_pkg speech_recognition.launch '"
  "roslaunch pepper_nodes pepper_bringup.launch"
  "top"
)

# Esegui ogni comando in un nuovo terminale
for cmd in "${commands[@]}"; do
  gnome-terminal -- bash -c "$cmd; exec bash"
done

#roslaunch your_package second_launch_file.launch &


wait # Wait for all background processes to finish
