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


---

## ✒️ Crafted by

**Sadegh-Kalami**

🔗 **Connect with me:**

- **GitHub Profile**: collaborations at [Sadegh-Kalami](https://github.com/Sadegh-Kalami).
- **Personal Site**: My projects and portfolio at [Sadegh-Kalami's GitHub Page](https://sadegh-kalami.github.io/).

*Your journey through my digital craftsmanship begins here.*

---