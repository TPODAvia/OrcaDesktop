# Robot periodic state (broadcast at ~2–5 Hz)
std_msgs/Header header           # frame_id: "map"
string robot_name
string map_name
string floor_name

uint8 mode                       # like your MODE_TABLE code
string mode_text

string current_task_id
string current_task_type         # "patrol"|"delivery"|"goto"|"return"|"" 
string current_vertex
string next_vertex

string[] path_vertices           # planned path (vertex names)
string[] reserved_vertices       # soft reservations for micro-coordination

geometry_msgs/Pose2D pose        # base_link in map (x,y,theta)
float32 battery_pct
float32 battery_voltage
float32 vehicle_speed
float32 internal_temp
float32 external_temp
float32 barometer

uint8 motor_status
uint8 lidar_status
uint8 imu_status
uint8 mag_status
uint8 gps_status
uint8 screen_status
uint8 camera_status
uint8 cpu_status
uint8 gpu_status
uint8 network_status
uint8 led_status
uint8 odom_status
uint8 map_status
uint8 nav_status
uint8 rmf_status
uint8 diagnostic_status

bool                         wcs_has_fix          # true if GPS fix available
float64                      wcs_lat              # degrees
float64                      wcs_lon              # degrees
float64                      wcs_alt              # meters (MSL). NaN if unknown.
geometry_msgs/Quaternion     wcs_orientation_enu  # IMU orientation (ENU)