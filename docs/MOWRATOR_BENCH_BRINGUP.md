# Mowrator bench hardware bring-up

Purpose: verify the new bench hardware one piece at a time before the mower is mounted, the ESCs are connected, or any motors can move.

## Current hardware snapshot

- The old main board has been removed.
- The old low-level board is no longer connected and is not part of the supported runtime. Its useful IMU role is replaced by a SparkFun LSM6DSO on Raspberry Pi I2C bus 1:
  - `SDA`: physical pin 3, `GPIO2 / SDA1`
  - `SCL`: physical pin 5, `GPIO3 / SCL1`
  - use Pi `3V3`, not `5V`, and a shared ground
- The Ardusimple ZED-F9P board is jumpered directly to the Pi. With the antenna disconnected, the first goal is only to prove data transport.
- A Teltonika RUTX11 is connected to the Pi Ethernet port and should provide cellular data only for the NTRIP/RTCM path.
- A Slamtec C1 is connected over USB for later SLAM work; for now only USB enumeration is in scope.

## Safety boundary

Do these checks with ESCs and motors disconnected or otherwise made physically unable to move. Keep `OM_ENABLE_MOWER=false` until the deliberate motor bring-up stage.

Do not use full autonomous launch as the first test. The first useful checks are host-level device checks, then narrow ROS topic checks.

The supported runtime uses `/hw/power`. Battery percentage and low-battery parking thresholds use the drive ESC voltage pair, while `/hw/power` also carries the mower/blade ESC voltage for cutoff logging. For bench-only checks with drive ESCs unpowered, set `OM_BATTERY_EMPTY_VOLTAGE=-1.0` and `OM_BATTERY_CRITICAL_VOLTAGE=-1.0` only if you need to run mower logic without valid ESC voltage. Restore the real 58.4V/45.0V/43.0V thresholds before any autonomous or field run.

## One-command bench check

On the Raspberry Pi checkout:

```bash
cd ~/open_mower_ros
RUTX_IP=192.168.1.1 \
GPS_PORT=/dev/ttyAMA2 \
GPS_BAUD=921600 \
LSM6DSO_BUS=1 \
LSM6DSO_ADDRESS=0x6B \
  ./utils/scripts/hardware/verify_bench_hardware.sh
```

Use `LSM6DSO_ADDRESS=0x6A` if the SparkFun address jumper was changed. Use a `/dev/serial/by-id/...` path for `GPS_PORT` if the F9P is connected by USB instead of Pi UART.

## RUTX11 cellular-only RTCM

First login/setup:

1. Connect the Pi Ethernet port to a RUTX11 LAN port.
2. Browse to the RUTX11 LAN address. Factory defaults are commonly `192.168.1.1`, user `admin`, and either `admin01` or the unique password printed on the device label.
3. Change the admin password immediately.
4. Configure the SIM/mobile WAN in the setup wizard or `Network -> WAN`.
5. Disable RMS/cloud features unless you explicitly need them; they can create background traffic.
6. Put the RUTX LAN on a subnet that does not conflict with the Pi's Wi-Fi LAN. A dedicated bench subnet such as `192.168.77.1/24` keeps route decisions obvious.
7. Set a very small mobile data limit while testing.

Routing rule of thumb:

- Wi-Fi remains the Pi default route for SSH, updates, web UI, and normal internet.
- Ethernet to the RUTX gets no default route.
- Only the NTRIP caster IP gets a host route through the RUTX gateway.
- The RUTX firewall should allow the Pi only to the caster IP and NTRIP port, then reject other Pi-to-mobile traffic.

On the Pi, make the Ethernet NetworkManager profile non-default:

```bash
nmcli -t -f NAME,DEVICE connection show --active
sudo nmcli connection modify "YOUR_ETH0_PROFILE" ipv4.never-default yes ipv6.never-default yes ipv4.route-metric 900
sudo nmcli connection up "YOUR_ETH0_PROFILE"
ip route
```

After this, the normal `default via ...` route should still point at Wi-Fi, not `eth0`.

Example Pi host route after the RUTX LAN is changed to `192.168.77.1`:

```bash
NTRIP_HOST="your-caster.example.com"
NTRIP_IP="$(getent ahostsv4 "$NTRIP_HOST" | awk 'NR==1 {print $1}')"
sudo ip route replace "$NTRIP_IP/32" via 192.168.77.1 dev eth0
ip route get "$NTRIP_IP"
```

Persist the route only after the caster IP and carrier behavior are known. If the caster uses changing IPs, prefer a firewall allow-list and refresh the host route during startup rather than making stale routes permanent.

RUTX firewall notes:

- Create the allow rule before the reject rule.
- Use IP addresses, not hostnames, in firewall rules.
- Allow only the NTRIP caster port, usually TCP `2101` unless your correction provider says otherwise.
- Use the RUTX mobile usage page to confirm only RTCM/NTRIP traffic is consuming cellular data.

## SparkFun LSM6DSO IMU

Host wiring check:

```bash
sudo raspi-config nonint do_i2c 0
sudo reboot
```

After reboot:

```bash
cd ~/open_mower_ros
./utils/scripts/hardware/read_lsm6dso.py --bus 1 --address 0x6B --samples 5
```

Expected result:

- `WHO_AM_I` reads `0x6c`.
- One accelerometer axis is near `+/-9.8 m/s^2` while the sensor is stationary.
- Gyro values are near zero while the sensor is still.

ROS integration:

```bash
export OM_USE_LSM6DSO_IMU=True
export OM_LSM6DSO_I2C_BUS=1
export OM_LSM6DSO_I2C_ADDRESS=0x6B
export OM_LSM6DSO_AXIS_CONFIG=+X+Y+Z
```

Then rebuild/restart the Pi runtime and verify:

```bash
rostopic echo -n 1 /hw/imu/data_raw
rostopic hz /hw/imu/data_raw
```

The axis config is a mounting calibration, not just a wiring setting. Leave it as `+X+Y+Z` for bench electrical checks, then set the final signed axis mapping once the IMU is mounted in the mower frame.

## Ardusimple F9P wiring check

If using UART:

- connect F9P `TX` to the Pi RX pin for the selected UART
- connect F9P `RX` to the Pi TX pin for the selected UART, otherwise RTCM cannot be sent back later
- connect ground
- set Ardusimple `IOREF` for Pi-compatible `3.3V` logic if using the Arduino-rail UART pins

With no antenna connected, expect serial traffic but no useful GNSS fix. That is fine for this stage.

Raw traffic check:

```bash
GPS_PORT=/dev/ttyAMA2 GPS_BAUD=921600 ./utils/scripts/hardware/verify_bench_hardware.sh
```

Success is any repeatable UBX/NMEA byte stream. No RTK fix is expected until the antenna and RTCM path are working.

Repo runtime note: the existing `Mowrator` profile still defaults GPS to `/dev/ttyAMA2` at `921600` baud. Override `OM_GPS_PORT` if the F9P ends up on USB or another UART.

## Slamtec C1 USB-only check

For now, only prove that the device enumerates:

```bash
lsusb
ls -l /dev/serial/by-id/
```

Do not add SLAM launch wiring until the base hardware, IMU, GPS transport, and correction path are stable.

## Vendor references

- Teltonika RUTX11 default login and setup wizard: [RUTX11 recovery/default login](https://wiki.teltonika-networks.com/index.php?mobileaction=toggle_view_desktop&title=RUTX11_Device_Recovery_Options), [RUTX11 setup wizard](https://wiki.teltonika-networks.com/wikibase/index.php?title=RUTX11_Setup_Wizard)
- Teltonika firewall and mobile usage pages: [Firewall traffic rules](https://wiki.teltonika-networks.com/view/Firewall_traffic_rules), [RUTX11 realtime data](https://wiki.teltonika-networks.com/view/RUTX11_Realtime_Data)
- SparkFun LSM6DSO product specs: [SparkFun LSM6DSO Qwiic](https://www.sparkfun.com/sparkfun-6-degrees-of-freedom-breakout-lsm6dso-qwiic.html?gQT=2)
- Ardusimple simpleRTK2B UART/IOREF notes: [simpleRTK2B Budget user guide](https://www.ardusimple.com/user-guide-simplertk2b-budget/)
