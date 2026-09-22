# WGS84 geographic pose.
# heading_deg is true-north clockwise [deg].

std_msgs/Header header
tc_geo_msgs/GeoPoint point

float32 heading_deg
bool has_heading

# Optional ENU yaw for conversion validation and debug display.
# East-zero, counter-clockwise positive [rad].
float32 yaw_enu_rad
bool has_yaw_enu

string child_frame_id
