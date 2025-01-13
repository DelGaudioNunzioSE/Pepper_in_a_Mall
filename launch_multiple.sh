#!/bin/bash

# Lista di comandi da eseguire
commands=(
  "cd project_ws && source devel/setup.bash && roslaunch ros_audio_pkg speech_recognition.launch"
  "cd project_ws && source devel/setup.bash && roslaunch rasa_ros dialogue.xml"
  "cd chatbot && rasarun"
)

# Esegui ogni comando in un nuovo terminale
for cmd in "${commands[@]}"; do
  gnome-terminal -- bash -c "$cmd; exec bash"
done
