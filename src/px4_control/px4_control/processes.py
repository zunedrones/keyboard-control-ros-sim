# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run QGroundControl
    "cd ~/Downloads && ./QGroundControl.AppImage",

    # Run the PX4 SITL simulation
    "cd ~/ros2_packs/PX4-Autopilot && make px4_sitl gz_x500",

    "ros2 run px4_control simple_controller"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)