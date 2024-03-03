# ROS Tutorial - Session 1

Welcome to the first session of our ROS tutorial! This guide is designed to help you understand and practice the basics of ROS (Robot Operating System).

## Accessing the Online ROS Platform
To access the online ROS environment, visit the following link and log in:
- [ROS Online Platform](https://app.reconstructinc.com/login)

## Session Overview
In this session, we will cover several fundamental ROS commands and concepts. We'll learn how to publish and subscribe to topics, activate nodes and packages, and use tools like RViz and rqt.

### Topics and Messages
- **Publishing a Topic:**
  Use the command below to publish a message of type `String` to a topic named `/my_topic`:
  ```bash
  rostopic pub /my_topic std_msgs/String "Hello"
  ```
- **Publishing Messages at a Specific Rate:**
  To publish a message at a rate of 100Hz, use the `-r` flag:
  ```bash
  rostopic pub -r100 /my_topic std_msgs/String "Hello"
  ```
- **Listing Active Topics:**
  To view a list of all active topics, use:
  ```bash
  rostopic list
  ```
- **Using Different Message Types:**
  Here's how to publish a `Float64` message:
  ```bash
  rostopic pub -r100 /my_topic std_msgs/Float64 2.0
  ```
- **Echoing Messages from a Topic:**
  To print messages being published on a specific topic, use `rostopic echo`. For example:
  ```bash
  rostopic echo /my_topic
  ```

### Nodes
- **Activating a Node:**
  To run a node, use the `rosrun` command:
  ```bash
  rosrun <Package> <Node>
  ```
  For instance:
  ```bash
  rosrun turtlesim turtlesim_node
  rosrun turtlesim turtle_teleop_key
  ```
  **Note:** "teleop" stands for teleoperation, controlling a system remotely.

### Packages
- **Activating Packages:**
  To launch a package, use the `roslaunch` command:
  ```bash
  roslaunch <Package> <File>
  ```

### RViz (ROS Visualization)
- **Starting RViz:**
  To open the RViz visualization tool, use:
  ```bash
  rviz rviz
  ```
  In RViz, you can set the Fixed Frame and use the 2D Nav Goal to send commands to the robot.

### ROS Graph and GUI Tools
- **Visualizing the ROS Graph:**
  To visualize the connections between nodes, topics, and messages, use:
  ```bash
  rqt_graph
  ```
- **Additional GUI Tools:**
  Other helpful GUI tools include:
  ```bash
  rqt_console
  rqt_bag
  ```

### Using ROSRUN with C++ and Python
- **Running ROS Nodes in Different Languages:**
  You can run ROS nodes written in Python or C++:
  ```bash
  rosrun rospy_tutorials listener
  rosrun rospy_tutorials talker
  ```
  Don't forget to check the active topics with `rostopic list`.

## Upcoming in Session 2
In our next session, we'll start node programming with Python, setting up and working with a Catkin workspace, and introducing robot modeling using URDF (Unified Robot Description Format). Stay tuned!

---

Feel free to reach out if you have any questions or need further clarifications on the topics covered. Happy learning!




```
```


# <mark>2. Session 2</mark>
make python node + debug + make workspace+ Grpah of nodes + get data and info of nodes.
what is ROS what can U do with ROS 
## make workspace directory
```bash
mkdir classROSws
cd classROSws
mkdir src
```
and then 
```
catkin_make
catkin build --> faster and doesnt need to be in the worksapce directory
```


## make package ROS
```bash
catkin_create_pkg <name of the package> dependecies

catkin_create_pkg testSubPub std_msgs rospy roscpp
```
<p>
package.xml: 
tozhihat dar morde package :
sazande + vabasetgiha + emial+ dependecies </p
>


```bash 
sadegh@sadegh-GE63-Raider-RGB-8SE:~/classROSws/src$ catkin_create_pkg testSubPub std_msgs rospy roscpp 
WARNING: Package name "testSubPub" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
Created file testSubPub/package.xml
Created file testSubPub/CMakeLists.txt
Created folder testSubPub/include/testSubPub
Created folder testSubPub/src
Successfully created files in /home/sadegh/classROSws/src/testSubPub. Please adjust the values in package.xml.
sadegh@sadegh-GE63-Raider-RGB-8SE:~/classROSws/src$ ls
CMakeLists.txt  testSubPub
sadegh@sadegh-GE63-Raider-RGB-8SE:~/classROSws/src$ cd testSubPub/
sadegh@sadegh-GE63-Raider-RGB-8SE:~/classROSws/src/testSubPub$ ls
CMakeLists.txt  include  package.xml  src

```


include for header files
src for .cc files

## make app

```
if __name__=='__main__':
# in code maduli vase code digeii nist va faqt khodehs run mishe

``` 

```
#!/usr/bin/env python3

import rospy







if __name__=='__main__':
  rospy.init_node('publihser_node')
  rospy.loginfo('Hello world')

```
after saving code
`chmod +x pub_python.py`

install ROS VSCODE extention



## Check for pakcge 

```bash
catkin_make
source devel/s
setup.bash  setup.sh    setup.zsh   share/      
source devel/setup.bash 
rosrun testSubPub pub_python.py 
[INFO] [1709212524.315141]: Hello world
[WARN] [1709212524.317080]: This is warning

```

## Structure of ROS workspace
`/classROSws/src/testSubPub/src$`
```
/<name of workspace>/src/<name of module>/src/



```


```
rosnode info /nodename
rqt_graph

```

# 3. Session3

URDF + Gazebo + node Controli + RVIZ
x





# todo 

write a node to subscribe to a spesefic node to watch it and  if addad 2 did addad 3 publish kone to hamon node 