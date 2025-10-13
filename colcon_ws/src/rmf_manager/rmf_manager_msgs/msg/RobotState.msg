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
