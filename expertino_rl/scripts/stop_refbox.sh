#!/bin/sh
source ~/rcll/rcll-get-started/setup.sh
echo ======================
echo Stopping refbox...
echo ======================
podman stop refbox refbox-frontend mqtt-broker simulator

echo ======================
echo Removing containers...
echo ======================
podman rm refbox refbox-frontend mqtt-broker
podman rm simulator