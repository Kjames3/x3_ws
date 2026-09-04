#!/bin/bash
echo "Waiting for INA226 to appear on I2C bus 0 at address 0x40..."
while true; do
  if i2cdetect -y -r 0 2>/dev/null | grep -q "40: 40"; then
    echo "SUCCESS: INA226 detected at 0x40!"
    break
  fi
  sleep 1
done
