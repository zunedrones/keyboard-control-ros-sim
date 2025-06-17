# keyboard-control-ros-sim
Simulação de controle de drone pelo teclado, para ros2.

1 - Install PX4
```bash
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```

2 - Install ros2 humble
```bash
cd
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

3 - Setup Micro XRCE-DDS Agent & Client
```bash
cd
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

4 - Install QGroundControl
```bash
cd
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
chmod +x ./QGroundControl-x86_64.AppImage
```

5 - Clone repository
```bash
cd
git clone https://github.com/zunedrones/keyboard-control-ros-sim.git
cd keyboard-control-ros-sim
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash && echo "source ~/keyboard-control-ros-sim/install/setup.bash" >> ~/.bashrc
```

6 - Run the project
```bash
ros2 launch px4_control px4_control.launch.py
```
After that, 4 commands will be executed:
* MicroAgent
* Init QGroundControl
* Simulation gazebo
* Node ros

7 - How to control the drone
In the terminal:
- space: arm/disarm
- t: takeoff
- l: land

## Atention!
Change your directory in the commands of src/px4_control/px4_control/processes.py