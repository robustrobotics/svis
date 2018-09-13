Header header
float64 timestamp_ros_rx  # [seconds] time usb message was received 
float64 timestamp_ros  # [seconds] timestamp in ros epoch
uint32 timestamp_teensy_raw  # [microseconds] timestamp in teensy epoch
float64 timestamp_teensy  # [seconds] timestamp in teensy epoch
uint8 count  # number of camera images (rolls over at 256)
uint64 count_total  # number of camera images (rolls over at 256)
