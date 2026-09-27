# MEF-Explore

## Communication-Constrained Multi-Robot Entropy-Field-Based Exploration

This project focuses on implementing a multi-robot exploration framework for exploring an unknown environment under communication constraints. The complete system is being developed step by step, where each individual component is implemented and tested before integrating it with the next stage.

**Student Name:** Pratyush Kumar Khillo  
**Roll No.:** 24255  
**Course:** ECS(323)  

## Current Implementation

The implementation is being developed in the following sequence:

1. **Map Generation**
   - Creation of a 2D occupancy grid environment
   - 10 × 10 grid with free cells, unknown cells, and obstacles
   - Initial placement of 3 robots in the environment

2. **Robot Communication and Movement**
   - Definition of robot positions and movement
   - Movement in four directions: UP, DOWN, LEFT, and RIGHT
   - Boundary and obstacle checking
   - Calculation of distance between robots
   - Communication based on the distance between robots

3. **Dynamic Communication Graph**
   - Representation of robots as nodes in a communication graph
   - Communication links are created dynamically based on the distance between robots
   - The graph changes as the robots move through the environment

4. **Local Map Generation**
   - Each robot maintains its own local map
   - Robots sense their surrounding cells
   - The sensed information is used to update the corresponding local map

5. **Map Sharing and Map Merging**
   - Communication is divided into high-communication and low-communication conditions
   - High communication allows robots to share their explored map information
   - Low communication allows limited information exchange, such as robot positions
   - Local maps are merged when communication conditions allow map sharing

6. **Frontier Detection**
   - Frontiers are identified from the locally explored maps
   - A frontier represents an unknown cell adjacent to an explored free cell
   - The detected frontiers are used as potential exploration targets

7. **Entropy Calculation**
   - Implementation of the entropy formulation used in the MEF-Explore approach
   - Calculation of frontier-related and robot-related entropy
   - Combination of these terms to obtain the total entropy

8. **MEF-Based Goal Selection**
   - Selection of exploration goals using the entropy-field-based strategy
   - Integration of frontier information, robot positions, and communication constraints

9. **Autonomous Multi-Robot Exploration**
   - Autonomous movement of the robots toward selected exploration goals
   - Continuous sensing, communication, map updating, and goal selection during exploration

10. **Simulation and Evaluation**
    - Complete simulation of the multi-robot exploration system
    - Evaluation of exploration performance under communication constraints
    - Analysis of the obtained results with respect to the MEF-Explore methodology

## Current Stage

At the current stage, the initial components of the framework have been implemented and tested manually. The map, robot movement, communication conditions, dynamic communication graph, local maps, map merging, and frontier detection have been developed and verified individually.

For these initial stages, different robot positions, movements, communication conditions, and map-sharing situations have been considered manually to understand and verify the behaviour of each component.

The entropy-based MEF-Explore strategy and autonomous exploration are being implemented progressively after the individual components are verified.

## Results

### Map Generation

A 10 × 10 grid environment was created with predefined obstacles, free cells, and three initial robot positions. The generated map provides the common environment used for testing the subsequent components of the framework.

### Robot Communication and Movement

Robot movement was implemented using four directions: UP, DOWN, LEFT, and RIGHT, with boundary and obstacle checking. The communication condition between robots was determined using the Euclidean distance and a predefined communication threshold. The test output shows the communication condition between different pairs of robots before and after movement.

### Dynamic Communication Graph

A dynamic communication graph was generated based on the current positions of the robots. Robots within the communication threshold are represented as connected nodes, while robots outside the communication range remain disconnected. The graph therefore changes according to the relative positions of the robots.

### Local Maps

Each robot maintains an independent local map initialized with unknown cells. The robots sense their current cell and neighbouring cells in the four cardinal directions, and the sensed information is used to update their respective local maps. The generated visualizations show the explored information available to each robot.

### Map Merging

Map merging was tested under different communication conditions. When two robots are within the defined communication range, their known map information can be shared and merged. When robots are outside the communication range, map merging is not performed. The resulting visualization shows the effect of map sharing between communicating robots.

### Frontier Detection

Frontier detection was performed on the local maps of all three robots. An unknown cell is identified as a frontier when it is adjacent to at least one explored free cell. The detected frontier cells are visualized on each robot's local map and represent potential regions for further exploration.

## Project Structure

<pre>
pratyush_24255/
│
├── README.md
├── requirements.txt
│
├── src/
│   ├── M01_Map.py
│   ├── M02_Robot_Communication.py
│   ├── M03_Dynamic_Graph.py
│   ├── M04_Local_Maps.py
│   ├── M05_Map_Merging.py
│   ├── M06_Frontier_Detection.py
│   ├── M09_Simulation.py
│   ├── M10_Test_Map_Merging.py
│   └── M11_Test_Frontier_Detection.py
│
├── results/
    ├── 01_map/
    ├── 02_robot_communication/
    ├── 03_dynamic_graph/
    ├── 04_local_maps/
    ├── 05_map_merging/
    └── 06_frontier_detection/
│
└── report/
</pre>

## Requirements

- Python 3.x
- NumPy
- Matplotlib

Install the required packages using:

```bash
pip install -r requirements.txt