# SupervisorNode

<p align="center">
  <img src="./docs/logo.JPG" width="100%" />
</p>

The [Supervisor Node](src/supervisor_node.cpp) is responsible for storing the overall health state of the vehicle through
a finite state machine, which here has been implemented using YASMIN ([GitHub](https://github.com/uleroboticsgroup/yasmin)) library.
The node's output is the state of the FSM and is published as a topic.
Within the repository, two additional nodes are included for demonstration purposes:
- [State Selector](src/state_selector.cpp), which simulates state transitions of the FSM invoked externally;
- [Failure Simulator](src/failure_simulator.cpp), that provides some possibily errors the system can face with, stimulating respective responses by the machine.

In addition, there is also a [Launch File](launch/demo.py) for calling the execution of all the three nodes together with just a single command. 

## Table of Contents

1. [Index](#index)
2. [Installation](#installation)
3. [Demos](#demos)
   - [Python](#python)
   - [Cpp](#cpp)
4. [YASMIN Viewer](#yasmin-viewer)
5. [Citations](#citations)

## Index
- [SupervisorNode](.)
    - [CMake File](./CMakeLists.txt)
    - [Documents](./docs)
        - [Logo](docs/logo.JPG)
        - [Specifications](docs/FSM-Specs.pdf)
    - [Launch Files](launch)
        - [Demo](launch/demo.py)
    - [Package XML](./package.xml)
    - [Source Files](src)
        - [Failure Simulator](src/failure_simulator.cpp)
        - [State Selector](src/state_selector.cpp)
        - [Supervisor Node](src/supervisor_node.cpp)
    
## Specs
### Definitions
- **Primary Driving Stack**: high-performance autonomous driving solution but prone to failures or potentially hazardous conditions;
- **Secondary Driving Stack**: less performant but more reliable and conservative autonomous driving solution;
- **Common Failure**: a condition where autonomous driving with the primary stack cannot proceed, in which:
    - a critical node becomes unresponsive (misses a certain number of deadlines);
    - there is an identification of a risky situation for the vehicle (loss of grip, steering commands leading to a certain collision, etc.);
- **Severe Failure**: a hardware component becomes unresponsive, so:
    - a failure in sensor or actuator drivers.

### States
The node implements the following states:
- **Idle [I]**: the node is active and awaits signals from the outside;
- **Manual [M]**: the vehicle is in manual driving mode:
    - no fault checks are performed in this state;
    - all driving commands from the primary and secondary stacks are ignored;
- **Active [A]**: The vehicle is in autonomous driving mode:
    - fault checks are performed in this state;
    - control is entrusted to the primary driving stack;
- **Emergency Takeover [ET]**: the vehicle is in a risky state:
    - control is entrusted to the secondary driving stack;
- **Emergency Stop [ES]**: the vehicle is unable to move autonomously:
    - driving commands are ignored, and the vehicle is stopped in place.
 
### Transition
The node implements the following state transitions:
- **(I) ←→ (M)**: service callable from the outside;
- **(M) ←→ (A)**: service callable from the outside;
- **(A) → (ET)**: a common fault occurs;
- **(ET) → (A)**: the common fault is resolved;
- **(ET) → (M)**: service callable from the outside;
- **(A, M, ET) → (ES)**: a severe fault occurs;
- **(ES) → (ET)**: the severe fault is resolved, and the node entered ES state from A or ET;
- **(ES) → (M)**: the severe fault is resolved, and the node entered ES state from M.

## Installation

```shell
# sourcing ros2 underlay (foxy distro in this case)
$ source /opt/ros/foxy/setup.bash

# clone packages 
$ cd ~/ros2_ws/src
$ git clone https://github.com/uleroboticsgroup/simple_node.git
$ git clone https://github.com/uleroboticsgroup/yasmin.git
$ git clone https://github.com/england00/SupervisorNode.git

# installing dependencies
$ cd yasmin
$ pip3 install -r requirements.txt
$ cd ../..
$ cd rosdep install --ignore-src --from-paths src -y -r

# building workspace
$ colcon build
```
