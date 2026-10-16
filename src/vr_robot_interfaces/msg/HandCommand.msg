std_msgs/Header header
int32 hand_id         # left_hand=0 / right_hand=1 /dual_hand=2
string preset_name       # open/close/middle
int32[] motor_positions  # 电机位置
int32[] motor_velocities # 电机速度
bool enabled             # 是否使能
