# Instructions to run the code:

* Clone the repository into `src` of your workspace 
* Go to the root of the workspace and execute `catkin_make`
* To run the example run the following command:
```
1)rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual3a.yaml /home/abhishek/enpm809b_ws/src/agile_robotics_industrial_automation/config/team_config_1.yaml

2)roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true

3)rosrun agile_robotics_industrial_automation qual1_node

```

# General tips 

* Create your own branch and work independently on it.
* Once you think your work is done, inform the team. 
* **Don't merge or commit to the master branch!**
* Please follow Google CPP guidelines while writing the code. 
* Try incorporating OOP concepts in the code right from the start because we will be building next projects on the top of this project. 
* This is not trivial so please start working on it as soon as possible.
