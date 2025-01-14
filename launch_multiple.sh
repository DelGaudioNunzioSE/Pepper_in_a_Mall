#!/bin/bash

# Lista di comandi da eseguire
commands=(
  "source devel/setup.bash && roslaunch ros_audio_pkg speech_recognition.launch"
  "source devel/setup.bash && roslaunch rasa_ros dialogue.xml"
  "source devel/setup.bash && roslaunch pepper_nodes pepper_bringup.launch"
  "source devel/setup.bash && roslaunch examples pepper_client.launch"
)

# Esegui ogni comando in un nuovo terminale
for cmd in "${commands[@]}"; do
  gnome-terminal -- bash -c "$cmd; exec bash"
done
