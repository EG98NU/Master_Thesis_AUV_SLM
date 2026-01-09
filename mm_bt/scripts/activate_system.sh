#!/bin/bash

echo "[activate_system]: Waiting for nodes to start..."
sleep 5

# echo "[activate_system]: Activating zeno_joystick..."
# rostopic pub -1 /zeno_joystick/setState rami_msgs/NodeStatus "node_status: 'ACTIVE'"

# sleep 1

echo "[activate_system]: Activating guidance_manager..."
rostopic pub -1 /raasl_stack/guidance/guidance_manager/setState rami_msgs/NodeStatus "node_status: 'ACTIVE'"

sleep 2

echo "[activate_system]: Activating gamma_bt..."
rostopic pub -1 /gamma_bt/setState rami_msgs/NodeStatus "node_status: 'ACTIVE'"

echo "[activate_system]: System activated!"

