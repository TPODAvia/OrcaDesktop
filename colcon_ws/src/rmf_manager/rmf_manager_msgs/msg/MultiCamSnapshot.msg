# Batch of 6 camera images + metadata

std_msgs/Header header
string ns
string robot_name

# Topics for each camera
string cam0_topic
string cam1_topic
string cam2_topic
string cam3_topic
string cam4_topic
string cam5_topic

string audio0_topic
string audio1_topic
string audio2_topic
string audio3_topic
string audio4_topic
string audio5_topic

# CameraInfo for each camera
string cam0_info
string cam1_info
string cam2_info
string cam3_info
string cam4_info
string cam5_info

# Overall detections/states for the whole batch
bool   human_detected
bool   animal_detected
bool   vehicle_detected
bool   danger_detected
string danger_text
float32 danger_score
