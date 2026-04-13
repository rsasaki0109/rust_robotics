#!/bin/bash
# Launch TurtleBot3 in Gazebo and connect rust_robotics nodes
# Usage: ./ros2_nodes/launch/run_gazebo_demo.sh

set -e

source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== RustRobotics Gazebo Navigation Demo ==="
echo ""
echo "Step 1: Starting TurtleBot3 Gazebo simulation..."
echo "  (This will open a Gazebo window)"
echo ""

# Launch Gazebo with TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
GAZEBO_PID=$!

echo "Waiting for Gazebo to start (15s)..."
sleep 15

echo ""
echo "Step 2: Starting rust_robotics path planner node..."
cd "$(dirname "$0")/../.."
./ros2_nodes/path_planner_node/target/debug/path_planner_node &
PLANNER_PID=$!
sleep 2

echo ""
echo "Step 3: Starting rust_robotics DWA planner node..."
./ros2_nodes/dwa_planner_node/target/debug/dwa_planner_node &
DWA_PID=$!
sleep 2

echo ""
echo "=== All nodes running ==="
echo "  Gazebo PID: $GAZEBO_PID"
echo "  Planner PID: $PLANNER_PID"
echo "  DWA PID: $DWA_PID"
echo ""
echo "To send a goal:"
echo "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo "    \"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}\""
echo ""
echo "Press Ctrl+C to stop all nodes"

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $DWA_PID $PLANNER_PID $GAZEBO_PID 2>/dev/null
    wait $DWA_PID $PLANNER_PID $GAZEBO_PID 2>/dev/null
    echo "Done."
}

trap cleanup SIGINT SIGTERM
wait
