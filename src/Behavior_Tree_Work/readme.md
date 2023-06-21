

This repo for behavior tree 3.8 with Groot and ROS

**Commands to run** 

1. make

```
catkin_make
```

2. run the application simple tree

```
cd build 
./simple_tree
```

3. run groot

  ```
  cd ~/bt3_ros_groot/src/Groot/build/devel/lib/groot
  ./Groot
  ```
3. monitor

```
real time monitor 
connect
```



there are two nodes till now : 

1. bt_node : contains main bt ros node and node_1.h contains bt nodes  

2. subscriber_condition_node: contains main bt ros node and subscriber_condition_node.h , subscriber_condition_node.cpp contains bt nodes  but as ros nodes

   subscriber_condition_node :consists of three nodes ***as in picture***

**Commands to run ROS Pkg nodes**  

1. bt_node

```
rosrun bt_ros_pkg bt_node
```

2. subscriber_condition_node

```
rosrun bt_ros_pkg subscriber_condition_node
```

![SubscriberConditionNodeTree](/home/wafaa/Desktop/SubscriberConditionNodeTree.png)

