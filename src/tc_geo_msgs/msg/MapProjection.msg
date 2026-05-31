# ENU/LLH projection parameters shared by route and localization components.

std_msgs/Header header

uint8 PROJECTION_UNKNOWN=0
uint8 PROJECTION_LOCAL_TANGENT_PLANE=1
uint8 PROJECTION_UTM=2
uint8 projection_type

string projection_id
string datum
string map_frame_id
string earth_frame_id

float64 origin_latitude
float64 origin_longitude
float64 origin_altitude
float32 map_yaw_offset_rad

string utm_zone
bool utm_north
