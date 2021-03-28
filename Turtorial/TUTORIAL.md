# Tutorial

### Eerst bestandstructuur maken (packagename + dependancy)
```bash
catkin_create-pkg beginner_tutorials std_msgs rospy roscpp
cd /catkin_ws
catkin_make
```

### Aparte terminal voor roscore 
```bash
roscore
```

### Rosrun simnaam simname_node in aparte terminal
```bash
rosrun turtlesim turtlesim_node
```

### Rosrun simnaam methode of commando?
```bash
rosrun turtlesim turtle_teleop_key
```

### Installeren voor graphs 
```bash
sudo apt-get install ros-indigo-rqt
sudo apt-get install ros-indigo-rqt-common-plugins
```

### Graph gebruiken 
```bash
rosrun rqt_graph rqt_graph
```

### Rostopic automatisch geinstalleerd
```bash
rostopic -h
rostopic echo /turtle1/cmd_vel
```

### Rostopic pub usage
```bash
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
- '[2.0 -> deze heeft invloed op hoe lang naar voor, 0.0, 0.0]' '[0.0, 0.0, 1.8 -> invloed over hoeveel draaien]

### Rosrun plot uitvoeren
```bash
rosrun rqt_plot rqt_plot
```
### Topic toevoegen
- Topic "/turtle1/pose/" toevoegen en + teken

### Rosservice list (for services)
```bash
rosservice list
```
### Rosservice call givenService
```bash
rosservice call /clear
```
### Rosservice type givenService | rossrv show (for more info about service)
```bash
rosservice type /spawn | rossrv show
```
### Then call service with right arguments
```bash
rosservice call /spawn 2 2 1.5 "tut"
```


### Rosparam list (for global params)
```bash
rosparam list
```
### Rosparam get and set
```bash
rosparam set /background_g 255
rosparam get /background_g 255
```
### Rosservice call /clear (voor visualisatie en set params defenitief maken)
```bash
rosservice call /clear
```

### Rosparam dump file.yaml (put params in a file.yaml)
```bash
rosparam dump params.yaml
```
### Rosparam load file simName
```bash
rosparam load params.yaml copy_turtle
```

### Add extra package for rqt
```bash
sudo apt-get install ros-indigo-rqt ros-indigo-rqt-common-plugins ros-indigo-turtlesim
```

### Use rqt_console (for debugging)
```bash
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```
- For servereties (DEBUG, INFO, WARN, ...)

### Roslaunch how?
```bash
cd ~/catkin_ws
source devel/setup.bash
roscd beginner_tutorials
```
### Create dir launch and navigate
```bash
mkdir launch
cd launch
```
### Create launch file
```bash
nano turtlemimic.launch
```
### Put code in file
```xml
<launch>

     <group ns="turtlesim1">
       <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
     </group>

     <group ns="turtlesim2">
       <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
     </group>

     <node pkg="turtlesim" name="mimic" type="mimic">
     <remap from="input" to="turtlesim1/turtle1"/>
     <remap from="output" to="turtlesim2/turtle1"/>
   </node>

</launch>
```

### Use roslaunch (top lvl)
```bash
roslaunch beginner_tutorials turtlemimic.launch
```
### Try it out
```bash
rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
### Use rst_graph to visualise
```bash
rqt_graph
```

### Using rosed kan van overal gebruikt worden
```bash
rosed rospy (spatie + dubbel tab)
```
### Choose a file to edit
```bash
export EDITOR='nano -w'
rosed rospy package.xml
```

### Verschil serv en msg
- msg geeft geen '---', serv wel '---'

*Voorbeeld msg*
```bash
int32 score
```
*voorbeeld serv*
```bash
int64 A
int64 B
---
int64 sum
```

### Add to package.xml
```xml
<build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

### Add to CMakeLists.txt in find_package()
```bash
message_generation
```
### Add to CMakeLists.txt in catkin_package()
```bash
CATKIN_DEPENDS message_runtime
```
### Add to CMakeLists.txt in add_message_files() en uncomment
```bash
add_message_files(
	FILES
	num.msg
)
```

### Add to CMakeLists.txt in generate_messages() en uncomment
```bash
generate_messages(
	DEPENDENCIES
	std_msgs
)
```

### Use rosmsg show
```bash
rosmsg show beginner_tutorials/Num
OR
rosms show Num
```

### Add srv to project + new srv
```bash
mkrdir srv
cd srv
roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

### Add to CMakeLists.txt in generate_messages() en uncomment
```bash
add_service_files(
	FILES
	AddTwoInts.srv
)
```

### Use rossrv show 
```bash
rossrv show beginner_tutorials/AddTwoInts
```

### Use rossrv list and rosmsg list
*Voorbeeld rossrv list*
```bash
rossrv list | grep 'somth'
```

*Voorbeeld rosmsg list*
```bash
rosmsg list | grep 'somth'
```

### Use catkin_make
```bash
cd ./catkin_ws
catkin_make
```

### Get script from gitlab
```bash
cd ./beginnertutorial
mkdir script
cd script/
wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
chmod +x talker.py
```

### Add to CMake 
```bash
rosed beginner_tutorials CMakeLists
```
- Then add
```bash
## Declare a Python interpreter
catkin_install_python(PROGRAMS script/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### Add listener script -> Same as before
```bash
roscd beginner_tutorials/scripts/
wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
chmod +x listener.py
rosed beginner_tutorials CMakeLists
```
- Then add

```bash
## Declare a Python interpreter
catkin_install_python(PROGRAMS script/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
### Use catkin_make
```bash
cd ./catkin_ws
catkin_make
```

### Before running subscriber
```bash
cd ~/catkin_ws
source ./devel/setup.bash
```

### Running publisher
*Open new terminal*
```bash
rosrun beginner_tutorials talker.py
```
*Open new terminal*
```bash
rosrun beginner_tutorials listener.py
```


## Writing a service node!
### Go to script and create a new scipt
```bash
nano add_two_ints_server.py
```
- Add this demo-code for server
```bash
#!/usr/bin/env python

from future import print_function

from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if name == "main":
    add_two_ints_server()
```

- Then chmod script server

```bash
chmod +x add_two_ints_server.py
```
- Add this demo-code for client
```bash
#!/usr/bin/env python

from future import print_function

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if name == "main":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
```

- Then chmod script client

```bash
chmod +x add_two_ints_server.py
```

- Add to CMakeLists.txt

```bash
rosed beginner_tutorials CMakeLists
```
- Then add script/add_two_ints_server.py and script/add_two_ints_client.py

```bash
catkin_install_python(PROGRAMS script/talker.py 
script/listener.py 
script/add_two_ints_server.py 
script/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### Use catkin_make
```bash
cd ./catkin_ws
catkin_make
```

## Use de services
### Niet vergeten de source terug aan te passen met source devel/setup.bash
*Terminal 1*
```bash
source devel/setup.bash
rosrun beginner_tutorials add_two_ints_server.py
```
*Terminal 2*
```bash
source devel/setup.bash
rosrun beginner_tutorials add_two_ints_client.py
```

## Install gazebo and its components
```bash
sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
```
