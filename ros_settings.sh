source /opt/ros/humble/setup.bash
source ~/ros2_humble_wss/thesis_wsp/install/setup.bash

export MY_ROBOT='mpo_700'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export Number_of_Robots=3

export MAP_NAME='my_map'
# export MAP_NAME='straight_road_map'
# export MAP_NAME='L1building'
# export MAP_NAME='L1building_static_obs_0'
# export MAP_NAME='L1building_dynamic_obs_0'
# export MAP_NAME='L1building_static_and_dynamic_obs_0'

# export MAP_NAME='neo_track1'
# export MAP_NAME='neo_workshop'

export GAZEBO_MODEL_PATH=~/.gazebo/models:~/ros2_humble_wss/thesis_wsp/install/neo_simulation2/share/neo_simulation2/models:$GAZEBO_MODEL_PATH

