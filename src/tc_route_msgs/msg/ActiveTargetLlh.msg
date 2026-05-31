# Current active target in WGS84 LLH for GUI, HTML UI, and logging.
# /active_target remains the ENU control target.

std_msgs/Header header

int32 route_version
int32 target_index
string target_label

tc_geo_msgs/GeoPose target_pose

float32 distance_m
float32 bearing_deg

bool is_avoidance_subgoal
string target_kind
