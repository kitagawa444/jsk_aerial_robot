# Observer zero-offset calibration; this does not reset the observer dynamics.
uint8 IDLE=0
uint8 COLLECTING=1
uint8 SUCCEEDED=2
uint8 REJECTED=3
uint8 FAILED=4
std_msgs/Header header
uint8 state
string detail
uint32 samples
# Last successfully applied baseline, retained on rejection/failure.
# Zero calibration_stamp means no baseline has been published yet.
time calibration_stamp
# Start of the collection that produced the last applied baseline.
time calibration_start_stamp
geometry_msgs/Wrench baseline_cog
string world_frame
geometry_msgs/Vector3 baseline_world_force
