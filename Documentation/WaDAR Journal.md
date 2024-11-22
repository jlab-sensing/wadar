
# Project WaDAR Notes

## Abstract
These are my notes taken during my work on the WADAR project for Dr. Colleen Josephson's jLab at UCSC.

## Table of Contents
1. [Project Description](#project-description)
2. [Chipotle Radar Communication](#chipotle-radar-communication)
    - [SSH Debugging](#ssh-debugging)
3. [frameLogger Debugging and Resolution](#framelogger-debugging-and-resolution)
    - [Debugging](#debugging)
    - [Resolution](#resolution)
4. [Backscatter Tag Assembly](#backscatter-tag-assembly)
    - [Old Tag w/ Eval Boards](#old-tag-w-eval-boards)
    - [Construction](#construction)
    - [Final Checks](#final-checks)
5. [Wireless Communication with Tragedy](#wireless-communication-with-tragedy)
    - [Communication Protocol](#communication-protocol-between-radar-and-tragedy)
    - [Connecting Personal Device](#connecting-personal-device-to-tragedy)
    - [Other Commands](#other-commands)
6. [GPS Coordinates](#gps-coordinates)

---

## Project Description
The WADAR (Water Radar) project aims to use underground backscatter tags paired with an ultrawideband radar as an above-ground reader to detect soil moisture in a low-cost low-power method.

---

## Chipotle Radar Communication
This chapter compiles information regarding communication with the Chipotle Radar computer (BeagleBone Black).

**Laptop password:** _kattilab_

```bash
# SSH into the BeagleBone.
ssh root@192.168.7.2

# Shutdown board (imperative, forgetting to do so will require re-flashing the BBB).
sudo shutdown -h now
```

### SSH Debugging
In order to correctly set up communication between a device and the radar, the SSH communication must work without requiring passwords. This is essential to allow the WADAR scripts to capture and transmit data automatically without repeatedly prompting for a password. Both devices must have each other's SSH keys. Some solutions to issues are listed below from my experience.

- The basic steps from https://www.strongdm.com/blog/ssh-passwordless-login can be followed for the most part.
- Adding `PubkeyAcceptedAlgorithms +ssh-rsa` to `/etc/ssh/ssh_config` when the SSH key isn't being accepted.
- Call `sudo systemctl restart ssh` to restart the SSH server after the config file is changed.
- Validate the setup by SSHing into the radar and SSHing back to our device without a password prompt.

---

## frameLogger Debugging and Resolution
This chapter documents the non-functionality of the radar after being re-flashed, and how it was debugged and solved.

### Debugging
The issue with `frameLogger.c` was identified at the line:
```c
status = radarHelper_configFromFile(rh, inFile_stage1, 1);
```

The variable `status` is not equal to `0`, resulting in the code terminating early. The function `radarHelper_configFromFile()` is from `radarHelper.h`.
```c
/**
   Helper function used to read in a user configuration JSON file and then set
   the radar configuration to match
   @param [in]  handle   Handle to the radar object
   @param [in] *path     Path to JSON configuration file
   @param [in] stage     Pre or post timing calibration register setup (1==pre calibration, 2==post calibration)
   @return 0 on success, otherwise 1 on failure
   @ingroup module_radarhelper
*/
```

After changing `inFile_stage1` from `stage1Chipotle.json` to `stage1.json`, this error message appears:
```c
ERROR 0x502: Unknown variable - PGSelect
    at: findVariable_ByName in src/support/Radarlib3_helpers.c:124
    from: NVA_VarGetIntProperties_ByName in src/Radarlib3.c:1015
```

### Resolution
The resolution required amending `frameLogger.c` using guidance from `sensorlogic.ai`.

Example JSON for `stage1.json` (Chipotle version):
```json
[
  {"DACMin" : 0},
  {"DACMax" : 8191},
  {"DACStep" : 32},
  {"Iterations" : 200},
  {"PulsesPerStep" : 32},
  {"FrameStitch" : 1},
  {"SamplingRate" : 0}
]
```

Additional fix in `frameLogger.c`:
```c
// Set the low frequency pulse generator
setIntValueByName(rh, "PulseGen", 1);
```

---

## Backscatter Tag Assembly

### Old Tag w/ Eval Boards
#### Overview
The following steps outline the required parts and step-by-step construction process for reassembling the backscatter tag.

#### Required Parts
- **Water-proof Container:** Polycase ML-46F
- **Teensy Microcontroller (Oscillator):** Adafruit Teensy
- **RF Switch:** NextWarehouse RF Switch
- **Antenna:** 5–18 GHz Vivaldi WA5VJB Antenna
- **Power Source:** Micro USB cable
- **Cable Glands:** GiBot Cable Glands
- **Antenna Mount:** 3D printed mount

#### Construction

1. **Prepare the Water-proof Container**
    - Drill a hole large enough for the PG11 cable gland.
    - Insert and secure the cable.
    - Test waterproofing.

2. **Mount the Oscillator and RF Switch**
    - Secure with tape and poster tack.
    - Add silica gel to keep the environment dry.

3. **Apply Static Shield**
    - Place a shield on the components to isolate the antenna.

4. **Install the Antenna**
    - Securely attach using a 3D printed mount.
    - Ensure alignment for optimal signal strength.

![Before antenna shift](images/beforeAntennaShift.png)
_Figure: Before antenna shift_

![After antenna shift](images/afterAntennaShift.png)
_Figure: After antenna shift_

#### Final Checks
Ensure:
- Components are secure.
- Waterproofing is effective.
- Antenna is isolated.

---

## Wireless Communication with Tragedy

### Communication Protocol Between Radar and Tragedy
```bash
# Verify radar connection via USB.
dmesg show

# List available IPs.
ip l

# Toggle enxs IPs on.
sudo IP link set enxXXXXXX up

# Use DHclient to assign IP.
sudo dhclient enxXXXXXX

# Validate SSH and set up pubkeys for passwordless access.
ssh root@192.168.7.2
ssh tragedy@192.168.7.1
```

### Connecting Personal Device to Tragedy
```bash
# Remove problematic driver to use Realtek WiFi modem.
sudo rmmod iwlmvm

# Re-enable the driver.
sudo modprobe iwlmvm
```

### Other Commands
```bash
# Show available IPs.
ip l

# Start WiFi advertiser.
systemctl start

# Interrogate fidelity.
ethtool

# Display IPs and configuration.
cat /etc/dnsmasq.d/quadruped

# Verify WiFi connection.
dmesg
```

---

## GPS Coordinates
This section pertains to the theory and methodology of working with GPS coordinates in the ROS package.

TODO

##
