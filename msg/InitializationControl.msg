# Optional, transport-independent authorization for stationary initialization.
# Publish periodically: permission expires when the publisher disappears.
uint8 BLOCKED=0
uint8 ALLOWED=1
uint8 RELEASED=2
uint8 ABORTED=3
std_msgs/Header header
uint8 state
string reason

string session_id
