# allow access to X server
xhost +local:all

# create Pulseaudio socket
pactl load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket

# set USB latency to 1ms for fast Dynamixel communication
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
