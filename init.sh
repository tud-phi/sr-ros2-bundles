# allow access to X server
xhost +local:all

# set USB latency to 1ms for fast Dynamixel communication
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
