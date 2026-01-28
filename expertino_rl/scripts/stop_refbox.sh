#!/bin/sh
source ~/rcll/rcll-get-started/setup.sh
echo ======================
echo Stopping refbox...
echo ======================
rc_stop

echo ======================
echo Removing containers...
echo ======================
podman rm refbox refbox-frontend mongodb simulator-frontend mqtt-broker
podman rm mongodb-check simulator