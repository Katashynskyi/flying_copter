# [Fly video link](https://youtu.be/zHgMqh8M3LE)
# Drone Navigation Script Documentation
## Overview

This Python script utilizes the DroneKit library to facilitate drone navigation. It consists of several key functions:

`arm_and_takeoff(target_alt)`
Arms the drone and initiates takeoff to a specified altitude.

`get_distance_mtr(location1, location2)`
Calculates the distance in meters between two geographic locations.

`get_bearing(location1, location2)`
Determines the bearing (azimuth) in degrees between two geographic locations.

`way_to_point(wpl)`
Navigates the drone to a specific geographic point.

`condition_yaw(yaw_need)`
Adjusts the drone's orientation (yaw) to a specific angle.
## Setup Instructions for Windows
#### Prerequisites

Mission Planner: Install Mission Planner.
PyCharm: Install PyCharm.

#### Git Clone

Clone the project repository:
`git clone git@github.com:Katashynskyi/temp_test.git`

#### Setting Up Environment

1. Open PyCharm:
    Go to File -> Open and open the project.
    Create a Conda 3.10 environment named temp_test. Refer to these instructions.

2. Change Environment:
    Set the environment to the newly created one.

3. Install Dependencies:
`pip install -r requirements.txt`

4. Resolve Error:
If an error occurs in dronekit/__init__.py, edit the file as follows:
Replace:
`collections.MutableMapping, HasObservers` 
With:
`collections.abc.MutableMapping, HasObservers`

#### Drone Simulation Setup

1. Start SITL:
Open the first terminal and execute:
`dronekit-sitl copter-3.3 --home=50.450739,30.461242,0,180`

2. Launch MAVProxy:
Open the second terminal and run:

`cd C:\Anaconda3\envs\temp_test\Lib\site-packages\MAVProxy
python .\mavproxy.py --master tcp:127.0.0.1:5760  --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551`

3. Run Mission Planner:
Launch Mission Planner and wait for MAVProxy to upload.

4. Execute Flight Script:
Open the third terminal and execute:
`python .\flight.py`

5. Additional Configuration:
While waiting for "ALT_HOLD" mode, switch the fly mode in Mission Planner from "Auto" to "AltHold" under the "Actions" tab if prompted.
Adjust the quadcopter UDP connection if necessary (right upper corner in Mission Planner).

## Conclusion

Following these steps will enable you to run the Python script for drone navigation using DroneKit.