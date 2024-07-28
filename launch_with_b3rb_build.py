import subprocess
from time import sleep
import sys
import os

kill_list=["ros2","vectors","detect","synapse","ruby","xterm","parameter","image","robot","planner","odom","vel","waypoint","controller","async_","bt_nav","behavior_","smoother","joy_node", "rviz2", "foxglove-studio"]


def main():
		cmd_input=sys.argv
		input_len=len(cmd_input)
		print(cmd_input,input_len)
		str="pkill "
		if cmd_input[1].lower() == "start" or cmd_input[1].lower() == "stop":
		    for i in kill_list:
		        str="pkill "+i
		        os.system(str)
		
		if cmd_input[1].lower() == "start":
		    sleep(1)
		    os.system("cd ~/cognipilot/cranium && colcon build --packages-select b3rb_ros_line_follower")
		    os.system("source ~/.bashrc")
		    sleep(1)
		    gz_subprocess=subprocess.Popen(["ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1"],stdout=subprocess.DEVNULL,shell=True) #starting simulation
		    ros_subprcoess3=subprocess.Popen(["ros2 launch electrode electrode.launch.py sim:=True"],stdout=subprocess.DEVNULL,shell=True) #starting 	runner node
		    sleep(1)
		    ros_subprcoess1=subprocess.Popen(["ros2 run b3rb_ros_line_follower vectors"],stdout=subprocess.DEVNULL,shell=True) #starting vectors node
		    ros_subprcoess2=subprocess.Popen(["ros2 run b3rb_ros_line_follower detect"],stdout=subprocess.DEVNULL,shell=True) #starting detect node

if __name__ == '__main__':   
    main() 
