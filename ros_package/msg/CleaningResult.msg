std_msgs/Header header

float32 x
float32 y
uint8 attempt_number

bool clean_successful
float32 confidence         # 0.0 – 1.0, how confident OpenCV is in the result
string failure_reason       # If clean_successful is false, this field may contain a reason for the failure (e.g., "object not detected", "cleaning mechanism malfunction", etc.)