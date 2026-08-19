#!/bin/bash
# Start the container.
#
#   ./run-docker-container.bash [jazzy|humble]
#
# Defaults to jazzy. Container name is ros-<distro>-toio-omni, so a jazzy and a
# humble container can coexist.
#
# BLE notes. The adapter is not passed through as a device: BlueZ runs on the
# host and everything happens over the system D-Bus socket mounted below, which
# is why --net=host and the socket mount are not optional here. The container
# user has the host UID (see the dockerfile) because the bus authenticates by
# UID. If a scan comes back empty but `bluetoothctl scan on` works on the host,
# check the socket mount before suspecting the sensor.
#
# The workspace is not persisted: this is an ament_python package and colcon
# builds it in a couple of seconds, so a fresh `colcon build` per container is
# cheaper than reasoning about a stale install/ directory.

file_dir=`dirname $0`
repo_dir=`cd ${file_dir}/.. && pwd`

# select ROS distro
distro=${1:-jazzy}
case "${distro}" in
  humble) codename=jammy ;;
  jazzy)  codename=noble ;;
  *)
    echo "Unsupported ROS distro '${distro}'. Use jazzy or humble." >&2
    exit 1
    ;;
esac

# start sharing xhost
xhost +local:root
user=`id -un`

if type nvidia-container-runtime >/dev/null 2>&1; then
  GPU_OPT="--gpus all"
fi

# Mount the dev kit's source tree too when it is checked out next to this repo,
# so the cube driver, the bringup and the sensor driver build into one
# workspace. The whole src/ rather than toio_ros2 alone: colcon crawls
# recursively, so anything added there is picked up without touching this file.
dev_kit_src="${repo_dir}/../toio_ros2_dev_kit/colcon_ws/src"
if [ -d "${dev_kit_src}" ]; then
  TOIO_OPT="-v `cd ${dev_kit_src} && pwd`:/home/toio/colcon_ws/src/toio_ros2_dev_kit"
  echo "mounting the dev kit sources from ${dev_kit_src}"
fi

# run docker
docker run -it --rm \
  --net=host \
  --ipc=host \
  ${GPU_OPT} \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/home/toio/.Xauthority \
  -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
  -v ${repo_dir}:/home/toio/colcon_ws/src/toio-omni-range-sensor \
  ${TOIO_OPT} \
  -e XAUTHORITY=/home/toio/.Xauthority \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --name "ros-${distro}-toio-omni" ${user}/ros-${distro}-${codename}-toio-omni
