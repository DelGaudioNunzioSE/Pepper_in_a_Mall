# Pepper as a Social Robot 🗣️
In this project, we programmed Pepper, the humanoid robot, to serve as a social companion capable of interacting naturally with users. Our
main objective was to enable Pepper to respond dynamically in two specific domains: shopping centers and customer PAR information, which
it could retrieve from a local database.

We used the ROS framework in order to manage the robot and RASA for the TOD module.

# HOW TO 🧑‍🏫

## HOW TO RUN

### First build
1. cd project_ws
- rm -rf build
- rm -rf devel
- rm -rf install
- rm -rf .catkin_workspace
2. catkin init
3. catkin build
4. source devel/setup.bash

### Then run the 4 package in different terminals
- source devel/setup.bash && roslaunch pepper_nodes pepper_bringup.launch (nao_ip:=x.x.x.x)
- source devel/setup.bash && roslaunch clients pepper_client.launch
- source devel/setup.bash && roslaunch rasa_ros dialogue.xml
- source devel/setup.bash && roslaunch ros_audio_pkg speech_recognition.launch

The first is the package about connection to pepper and launch of the server
The second one is about the client of the servers
The third one is abaout RASA nodes and RASA server
The last one is for the input audio 


## HOW TO INSTALL _________________________________________
### Preparation
1. sudo apt update
2. sudo apt install python3-pip

### Installing ROS
#### Setup sources.list and keys
1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
2. sudo apt install curl 
3. curl curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
#### Installation and environment setup
2. sudo apt install ros-noetic-desktop-full
3. source /opt/ros/noetic/setup.bash
4. echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc source ~/.bashrc
#### To use with python3
1. sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
2. sudo apt install python3-rosdep
3. sudo rosdep init
4. rosdep update
5. sudo apt install ros-noetic-catkin


### Installing catkin
1. sudo-apt-install python3-catkin-tools
2. sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential


### Installing Rasa 

3. pip3 install pip
4. pip3 install rasa
5. pip3 install rasa\[spacy\]
6. pip3 install rasa\[transformers\]

### Installing audio module: 
1. sudo apt install -y libasound-dev ffmpeg portaudio19-dev libportaudio2 libportaudiocpp0
2. python3 -m pip install pyaudio speechrecognition librosa sounddevice python_speech_features scipy soundfile

### Installing detection module:
1. sudo apt install -y ros-noetic-vision-msgs
2. python3 -m pip install tensorflow rosnumpy


### Installing NaoQi (Pepper)
1. sudo apt update && sudo apt install -y git
2. cd$HOME
3. wget https://github.com/gdesimone97/cogrob_pepper_nodes/releases/download
/requirements/install.bash -O install.bash
4. bash install.bash
5. cd $HOME/catkin_ws
6. catkin build
7. source devel/setup.bash

![Alt text](https://corporate-internal-prod.aldebaran.com/themes/custom/softbank/images/360/pepper.png)
