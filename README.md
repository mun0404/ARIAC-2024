# RWA3 Group 1
#### Authors: 
- Koushik Alapati (UID: 120239489) 
- Charith Gannapu Reddy (UID: 119360347)
- Akhil Javvadi (UID: 120147517)
- Mohammed Munawwar (UID: 120241642)
- Varad Nerlekar (UID: 120501135)

### [Github Repository Link](https://github.com/CharithReddy101/rwa3_group1)
---
## Introduction
This project contains the Tasks 1-8 for RWA 3 based on ARIAC (Agile Robotics for Industrial Automation Competition) v2023.5.0
<sup>[ARIAC 2023 Documentation](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/index.html)</sup>

The project is written in Python programming language and covers the following tasks:

#### Task 1:
Design an architecture for a CCS (Competitor Control System).
- Provide outline of the chosen architecture and the rationale behind it.
- Provide an activity diagram for the system.

##### A Document has been provided inside ```doc``` folder inside the project package named: ```RWA3_Group1_Report.pdf```.

#### Task 2:
Generate the package ```rwa3_<group#>``` with the given file structure.

#### Task 3:
- Create a service client for ```/ariac/start_competition```.
- Create a subscriber to ```/ariac/competition_state```.
- Verify within the subscriber callback whether the received messages indicate that the system is in the ```READY``` state prior to invoking the service client to initiate the competition.

#### Task 4:
Listen to messages on ```/ariac/orders``` and store published orders in data structures.

#### Task 5:
- Once an order is received and stored (see task #4), wait for 15 seconds before shipping the order (described in task #6).
- You will receive the first order as soon as you start the competition. Store this order and wait 15 seconds (𝑡0).
- While awaiting, a second order of higher priority is received. In adherence to priority in ARIAC, this new order must be fulfilled prior to the first. Consequently, a 15-second delay (𝑡1) is mandated before shipping and submitting this second order.
- Upon successful submission of the high-priority order, proceed to ship and submit the initial order. Ensure a lapse of 15 seconds for 𝑡0 before shipping the first order.

#### Task 6:
- Create service clients for the services described above and call them as needed.
- Based on the discussion from task #5, we should see AGV #3 being shipped before AGV #2.
- The AGV ID should be obtained dynamically from the order rather than being hardcoded.

#### Task 7:
- Create a service client for ```/ariac/submit_order```.
- Ensure the AGV has reached the warehouse by listening to ```/ariac/agv{n}_status```.
- Call the service client once the AGV is at the warehouse.

#### Task 8:
- Create a client for ```/ariac/end_competition```.
- Check that your CCS has shipped and submitted all orders. This can be done by checking that your data structure of orders is empty, for instance.
- Check there are no more orders to process (competition state is
```ORDER_ANNOUNCEMENTS_DONE```). This information is provided on ```/ariac/competition_state```.
- If both steps 2. and 3. are satisfied, call ```/ariac/end_competition``` to end the competition.

## File structure
```
rwa3_group1
├── CMakeLists.txt
├── doc
│   ├── README.txt
│   └── RWA3_Group1_Report.pdf
├── include
│   └── rwa3_group1
├── launch
│   └── rwa3_group1.launch.py
├── package.xml
├── rwa3_group1
│   ├── \_\_init\_\_.py
│   ├── kitting_interface.py
│   └── main_interface.py
├── script
│   ├── \_\_init\_\_.py
│   ├── kitting_node.py
│   └── main_node.py
└── src
```


## Requirements
- [Python 3.8.10](https://realpython.com/installing-python/)
    - threading
    - enum
- [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation.html)
- [ARIAC Framework](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/getting_started/installation.html)

## Usage

Make sure you have created a copy of ```rwa3_spring2024.yaml``` and placed it in the given location:
```
<your_ariac_workspace_directory>/src/ariac/ariac_gazebo/config/trials/
```

[Note]: Ensure that you have built and sourced ariac environment properly. If not, follow these steps:
1. ```cd <your_ariac_workspace>```
2. ```colcon build```
3. ```source install/setup.bash```

To run the ARIAC simulation, enter:
```
ros2 launch ariac_gazebo ariac.launch.py trial_name:rwa3_spring2024
```

This launches the gazebo environment for ARIAC 2023.

To launch the RWA3 Group1 submission, in a new terminal, run:
```
ros2 launch rwa3_group1 rwa3_group1.launch.py
```

[Note]: Ensure that the new terminal has also sourced the RWA3_Group1 package. You can source it using:
1. Place the 'rwa3_group1' ros2 package inside the ```<your_ariac_workspace>/src``` folder.
2. Follow the steps mentioned above to build and source the package.

--- End of file ---