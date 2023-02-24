# Goals sent to nearby drones

# Two types of actions:
# Examine: check whether in other drones' map the goals are informative or not
# Inform : inform other drones the examined goals 
uint8 action
float64 inform_time
uint16 drone_id
geometry_msgs/Point[] points
float32[] yaws