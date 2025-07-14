# Status constants used to denote the state of the monitored node
int8 NOMINAL = 1  # The node is functioning normally
int8 WARNING = 2  # The node is partially failing but still functioning
int8 ERROR = 3    # The node is no longer functioning
int8 NO_HB = 4    # The node has not been seen for long enough that it is considered missing
int8 STARTUP = 5  # The node is still initializing or has not been seen yet

# The human readable name for the node being monitored
string nickname
# The fully qualified ROS node name for the node being monitored
string node_name
# The status of the monitored node
int8 status
# Any notes about the status that the monitor wishes to send
string notes
# Last time the node was seen. This is usually set by the aggregator and/or used by forwarding
int64 last_heartbeat -1
# Whether the node is required for the system to function.
# This is usually set by the aggregator and/or used by forwarding
bool required true
