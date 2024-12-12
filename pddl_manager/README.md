# pddl_manager
PDDL manager node to create PDDL instances and to interact with it via ROS interfaces.

It is geared towards supporting executive frameworks and at the moment only supports a subset of features.
In particular, it currently is geared towards temporal domains.

## Prerequisites
This package has indexed dependencies as listed in the `package.xml` filethat can be installed via `rosdep`.
Aside from that it additionally requires the [unified-planning](https://unified-planning.readthedocs.io/en/latest/) (source code on [GitHub](https://github.com/aiplan4eu/unified-planning)) python framework. It can be installed via pip:
```bash
pip install unified-planning
```

## Usage
Run the provided lifecycle node using the provided launch file to automatically configure the node:
```bash
ros2 launch pddl_manager pddl_manager.launch.py
```
Alternatively, the unconfigured node can be used directly:
```bash
ros2 run pddl_manager pddl_manager.py
```

## Features
The `pddl_problem_manager` node offers various ROS interfaces.
### Services
 - **/add_pddl_instance**: adds a pddl instance and gives it a name.
 - **/add_fluent**: add a regular fluent to a pddl instance
 - **/get_fluents**: gets all fluents of a given pddl instance.
 - **/get_functions**: gets all current function values of a given pddl instance
 - **/check_action_precondition**: ground an action and check it's precondition against the current values in a given pddl instance. If it is not satisfied, it also provides the unsatisfied conditions.
 - **/get_action_effects**: ground an action and provide it`s effect.
### Actions
 - **/pddl_plan**: plan the given pddl instance using nextFLAP.
