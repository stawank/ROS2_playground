# ROS2 Playground

**ROS 2 Distro**: Jazzy  
**Target Environment**: Ubuntu 24.04 (Linux)  

---

##  Description

This repository is a **playground for ROS 2 experiments**, demos, and prototyping.  
It serves as a space to:

- Practice ROS 2 concepts (nodes, topics, services, actions, parameters)  
- Build and test example packages  
- Explore configurations and integration patterns  
- Create templates for future ROS 2 projects  




---

##  Contents

- Example publisher / subscriber nodes  
- Service and action demos  
- Sample launch files for testing multiple nodes  
- Parameter and config examples  
- A structured environment to quickly try out new ROS 2 ideas  

---

##  Prerequisites

Ensure the following are installed on your system:

- Ubuntu **24.04 LTS**  
- **ROS 2 Jazzy** (`/opt/ros/jazzy`)  
- **colcon** build system  
- **rosdep** for dependency management  

---

##  Usage

1. **Clone the repository**  
   ```bash
   git clone https://github.com/stawank/ROS2_playground.git
   cd ROS2_playground

2. **Source ROS**
   ```bash
    source /opt/ros/jazzy/setup.bash

3. **Install dependencies**
   ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

4. **Build the workspace**
   ```bash
    colcon build --packages-select turtlesim_catch_them_all

5. **Source the overlay**
   ```bash
    source install/setup.bash