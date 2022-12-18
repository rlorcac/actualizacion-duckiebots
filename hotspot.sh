#!/usr/bin/env bash
# This file enables/disables a hotspot named 'vehicle_name-wifi'

# if the vehicle name is not set, exit
if [ -z "$VEHICLE_NAME" ]; then
  echo "VEHICLE_NAME is not set. Please run 'source set_vehicle_name.sh' before enabling the hotspot"
  exit 1
fi
 
# vehicle name is set, create hotspot variable and enable or disable hotspot with nmcli
HOTSPOT="$VEHICLE_NAME-wifi"

case $1 in
  on)
    echo "Enabling hotspot $HOTSPOT..."
    if ! sudo nmcli connection up $HOTSPOT; then
      echo "Creating hotspot $HOTSPOT"
      sudo nmcli device wifi hotspot con-name $HOTSPOT ssid $HOTSPOT band bg password quackquack
    fi
    echo "$HOTSPOT enabled."
    ;;
  off)
    echo "Disabling hotspot $HOTSPOT..."
    sudo nmcli connection down $HOTSPOT
    echo "$HOTSPOT disabled."
    ;;
  *)
    echo "Usage: $0 {on|off}"
    exit 1
esac
