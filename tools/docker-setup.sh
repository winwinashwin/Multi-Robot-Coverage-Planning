#!/bin/bash -e

XAUTH=/tmp/.docker.xauth

if [[ -d $XAUTH ]]; then
  echo "[ FAIL ] Directory $XAUTH exists, delete and rerun setup."
  exit 1
fi

if [[ ! -f $XAUTH ]]; then
  echo "[ INFO ] File $XAUTH does not exist, created."
  touch $XAUTH
else
  echo "[ INFO ] File $XAUTH exists, will be overwritten"
fi

xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "[ INFO ] Configured Xauthority file: $XAUTH"
