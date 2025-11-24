# F450 + Pixhawk + Raspberry Pi 3B+ (MAVLink / DroneKit)

A complete, reproducible build of an autonomous quadcopter using an **F450** frame, **Pixhawk 2.4.8** (ArduCopter), and a **Raspberry Pi 3B+** companion computer over **MAVLink**. This repo focuses on **wiring and software bring-up** (DroneKit + MAVProxy), with example scripts for GUIDED flight, mission upload, and telemetry logging.

> ⚠️ **Safety first:** Always do bench tests with **props off**. Use a clear outdoor test area, and set appropriate **failsafes**.

---

## What you’ll build

- A Pixhawk-based quad on an F450 airframe with GPS/compass, 30A ESCs, 10×4.5 props.
- A Raspberry Pi 3B+ connected to Pixhawk **TELEM2** via UART, streaming MAVLink.
- Python scripts (DroneKit) for connect→arm→takeoff→navigate; plus logging and MAVProxy utilities.

Brief frame/parts notes are in `docs/assembly.md` (F450, 2212 motors, BLHeli-S ESCs, FS-i6X/IA6B, etc.). 

---

## Hardware overview

- **Airframe:** F450 X-quad, ~280 g, typical 3–4S Li-Po with ~10–20 min endurance. 
- **Autopilot:** Pixhawk 2.4.8 (ArduCopter).  
- **Companion computer:** Raspberry Pi 3B+ (**3.3 V TTL UART**).  
- **Telemetry/RC:** FS-i6X + IA6B (≥ 500 m LOS), optional 915 MHz telemetry radio 

---

## Wiring (Pixhawk ⇄ Raspberry Pi)

**Goal:** Serial MAVLink over Pixhawk **TELEM2** ↔ Pi UART.

- Connect **Pixhawk TELEM2 TX** → **Pi RX (GPIO15)**, **TELEM2 RX** → **Pi TX (GPIO14)**; **GND↔GND**. Keep wiring short; both sides use 3.3 V logic (don’t power Pixhawk from Pi’s 5 V). ArduPilot’s companion-computer guide shows the TELEM2↔Pi wiring for MAVLink. 
- If you prefer a USB-serial dongle instead of GPIO UART, use FTDI/CP2102 and connect TELEM2 to the adapter; it will appear as `/dev/ttyUSB*`.

See `docs/wiring.md` for a pinout diagram and photos.

---

## Raspberry Pi setup (OS, UART, Python, MAVLink)

1. **Enable UART and disable the serial login shell**
   - `sudo raspi-config` → **Interface Options** → **Serial Port** → _Login shell over serial? **No**_ → _Enable serial port hardware? **Yes**_. Reboot.
   - After reboot, the primary UART is available at **`/dev/serial0`** on a Pi 3B+.

2. **Serial permissions**
   - Add your user to the dialout group:
     ```bash
     sudo usermod -aG dialout $USER
     ```
     Log out/in (or reboot). 

3. **Python environment (DroneKit + MAVLink)**
   - Use Python **3.9** (DroneKit is not maintained for latest 3.11+). Create a venv and install:
     ```bash
     python3.9 -m venv ~/dronekit-py39
     source ~/dronekit-py39/bin/activate
     pip install --upgrade pip
     pip install dronekit pymavlink future pyserial
     ```

4. **MAVProxy (handy CLI for testing)**
   - Install MAVProxy inside the venv (or system-wide), then connect:
     ```bash
     # inside the venv
     mavproxy.py --master=/dev/serial0 --baudrate 57600 --aircraft MyCopter
     ```
     This opens a MAVLink console and modules (status, param, etc.). 

---

## Pixhawk configuration (AUX motor mapping & tests)

If your motors are wired to **AUX** outputs (not MAIN), map AUX1..AUX4 to motors 1..4 and ensure AUX pins are PWM outputs:

```text
param set BRD_PWM_COUNT 4
param set SERVO9_FUNCTION 33
param set SERVO10_FUNCTION 34
param set SERVO11_FUNCTION 35
param set SERVO12_FUNCTION 36
reboot
```

Run these in MAVProxy’s prompt (or via a GCS). Then test each motor safely (props off):
```text
motortest 1 1 1200 5
motortest 2 1 1200 5
motortest 3 1 1200 5
motortest 4 1 1200 5
```

The above mapping directs AUX1..AUX4 to Motor1..Motor4. For deeper background on output functions and the BRD_PWM_COUNT setting, see ArduPilot’s docs. Note: in newer firmware (4.2+), GPIO vs PWM selection relies more on SERVOx_FUNCTION values.


Create scripts/connect_pixhawk.py:
```python
#!/usr/bin/env python3
from __future__ import print_function
import time
from dronekit import connect

# Use Pi's UART. Match Pixhawk TELEM2 baud (commonly 57600 on ArduCopter)
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True, timeout=60)

print("Connected. Firmware:", vehicle.version)
print("GPS:", vehicle.gps_0, "  EKF OK:", vehicle.ekf_ok)
print("Mode:", vehicle.mode.name, "  Armable:", vehicle.is_armable)

# Stream a few heartbeats
for _ in range(5):
    print("HB: altitude=%.1f m  groundspeed=%.1f m/s  batt=%.1f%%" %
          (vehicle.location.global_relative_frame.alt,
           vehicle.groundspeed,
           (vehicle.battery.level or -1)))
    time.sleep(1)

vehicle.close()
```

Run it inside your venv:
```bash
sudo ~/dronekit-py39/bin/python scripts/connect_pixhawk.py
```


Guided demo: arm & takeoff

Create scripts/arm_and_takeoff.py:
```python
#!/usr/bin/env python3
from __future__ import print_function
import time, math
from dronekit import connect, VehicleMode
from pymavlink import mavutil

def wait_until_armable(v):
    while not v.is_armable:
        print("Waiting for EKF/IMU/GPS... mode=%s fix=%s sat=%s" %
              (v.mode.name, getattr(v.gps_0,'fix_type',None), getattr(v.gps_0,'satellites_visible',None)))
        time.sleep(1)

def set_mode(v, name):
    v.mode = VehicleMode(name)
    while v.mode.name != name:
        time.sleep(0.2)

def arm_and_takeoff(v, alt_m):
    wait_until_armable(v)
    set_mode(v, "GUIDED")
    v.armed = True
    while not v.armed:
        time.sleep(0.2)
    v.simple_takeoff(alt_m)
    # wait to reach ~95% of target altitude
    while True:
        alt = v.location.global_relative_frame.alt or 0.0
        print(" Alt: %.1f" % alt)
        if alt >= 0.95 * alt_m:
            break
        time.sleep(0.5)

def main():
    v = connect('/dev/serial0', baud=57600, wait_ready=True, timeout=60)
    print("Connected:", v.version)
    arm_and_takeoff(v, 5.0)
    print("Holding position for 10 s ...")
    time.sleep(10)
    print("RTL")
    set_mode(v, "RTL")
    time.sleep(2)
    v.close()

if __name__ == "__main__":
    main()
```

This follows the standard pattern: GUIDED → arm → simple_takeoff(), then switch mode or command further movement.

## Using MAVProxy alongside your scripts

You can keep MAVProxy connected for diagnostics while running Python scripts. It’s great for quick param edits, EKF checks, and running motortest.


## Bring-up checklist

RC + failsafes: Verify radio failsafe, geofence, RTL altitude.
Compass + accel calibrations: Complete in a GCS before flight.
Telemetry sanity: GPS, EKF OK, battery, and mode should look correct in connect_pixhawk.py output.
Bench tests (props off): Use motortest to confirm mapping/rotation before arming.


## Troubleshooting

No serial link: Ensure raspi-config serial login is disabled and UART enabled; confirm /dev/serial0 exists; try 57600/115200; swap RX/TX if needed; verify TELEM2 set to MAVLink.
Permission denied on /dev/serial0: Add user to dialout. 
Motor outputs on AUX: Re-apply SERVO9..12_FUNCTION and confirm BRD_PWM_COUNT.

## Contact
For queries please email me or open an issue or submit a pull request.
