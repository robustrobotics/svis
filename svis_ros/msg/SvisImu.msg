Header header
int8 SIZE=3
float64[3] timestamp_ros_rx  # [seconds] time usb message was received
float64[3] timestamp_ros  # [seconds] timestamp in ros epoch
uint32[3] timestamp_teensy_raw  # [microseconds] timestamp in teensy epoch
float64[3] timestamp_teensy  # [seconds] timestamp in teensy epoch
float32[3] accx  # [m/s^2]
float32[3] accy  # [m/s^2]
float32[3] accz  # [m/s^2]
float32[3] gyrox  # [rad/s]
float32[3] gyroy  # [rad/s]
float32[3] gyroz  # [rad/s]
