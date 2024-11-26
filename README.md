# Beispielanwendung für ds-crazyflies

## Installation

1. `source ds-crazyflies/install/setup.bash`
2. `colcon build`
3. `source install/setup.bash`

## Ausführen

Passe das [tha_framework.launch.py](tha_example/launch/tha_framework.launch.py) an um id, channel, intialposition und type festzulegen.

Terminal1: 
 - `source install/setup.bash`
 - `ros2 launch tha_example tha_framework.launch.py`

Terminal2:
 - `source install/setup.bash`
 - `ros2 run tha_example tha_flie`

Das Beispiel [tha_flie](tha_example/tha_example/tha_flie.py) lässt die Crazyflie abheben und fliegt an einen vorgegebenen Punkt. Anschließend fliegt diese wieder zur Heimat zurück und landet dort.