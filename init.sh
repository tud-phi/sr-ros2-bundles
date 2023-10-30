# set environment variable with current user and group id
export CURRENT_UID=$(id -u):$(id -g)

# allow access to X server
xhost +local:all

# create Pulseaudio socket
pactl load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket
# copy the config file to /tmp/pulseaudio_clientclient.conf
cp ./utils/pulseaudio/client.conf /tmp/pulseaudio_client.conf

# set USB latency to 1ms for fast Dynamixel communication
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
