uint8 SOURCE_NONE=0
uint8 SOURCE_VISION=1
uint8 SOURCE_FUSED=2
uint8 SOURCE_LIDAR=3

std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
float32 confidence
uint8 source
bool stable
uint32 accepted_views
uint32 lidar_support
string state
geometry_msgs/Vector3 bearing
bool bearing_valid
