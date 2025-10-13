# Dashboard -> Robots (topic-based, domain-bridge friendly)
# Only the command matching robot_name OR "*" is acted on.
string robot_name

uint8 type
# 0=GOTO, 1=PATROL, 2=DELIVERY, 3=CANCEL_TASK, 4=CANCEL_ALL,
# 5=RETURN_HOME, 6=SET_RESUME_AFTER_CHARGE

# for GOTO:
string goal

# for PATROL:
string[] waypoints
uint32 loops

# for DELIVERY:
string pickup
string dropoff
bool wait_for_trigger

# general:
string task_id
bool   resume_after_charge
