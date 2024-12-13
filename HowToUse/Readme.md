# Installing ROS
### Setup sources.list and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |
sudo apt-key add -
### Installation and environment setup
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc source ~/.bashrc
### To use with python3
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt-get install ros-noetic-catkin python-catkin-tools

# Installing Rasa 
sudo apt update
sudo apt install python3-pip
pip3 install pip
pip3 install rasa
pip3 install rasa\[spacy\]
pip3 install rasa\[transformers\]

# Installing audio module: 
bash audio_install_reqs.bash

# Installing detection module:
bash detection_install_reqs.bash

# audio References:
1. VAD settings: https://github.com/Uberi/speech_recognition/blob/1b737c5ceb3da6ad59ac573c1c3afe9da45c23bc/speech_recognition/__init__.py#L332
2. SpeechRecognition library documentation: https://pypi.org/project/SpeechRecognition/
3. 'Whisper' is built into the library, check the documentation attached above to understand how to configure it
