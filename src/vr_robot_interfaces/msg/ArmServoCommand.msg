uint8 MODE_POSE=0
uint8 MODE_JOINT=1

std_msgs/Header header
string arm_name
uint8 mode
geometry_msgs/Pose target_pose
float64[] joint_positions   
bool enabled
