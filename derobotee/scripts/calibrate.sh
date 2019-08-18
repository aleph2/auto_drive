rostopic pub -1 /twist_mux/cmd_vel geometry_msgs/Twist '{linear: {x: 0.05, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
sleep 3s
rostopic pub -1 /twist_mux/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
rostopic pub -1 /twist_mux/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.3}}'
sleep 3s
rostopic pub -1 /twist_mux/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.3}}'
sleep 3s
rostopic pub -1 /twist_mux/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'
rostopic pub -1 -l /boundary geometry_msgs/Polygon "{points:[{'x':0.76569211483,'y':0.0528378486633,'z':0},{'x':0.760314643383,'y':-2.0,'z':0},{'x':-2.3,'y':-2.0,'z':0},{'x':-2.3,'y':0.0323663949966,'z':0}]}"
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "map" }, pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
