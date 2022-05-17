roslaunch franka_gazebo panda.launch x:=-0.5 \
    world:=$(rospack find franka_gazebo)/world/stone.sdf \
    controller:=cartesian_impedance_example_controller \
    rviz:=true
