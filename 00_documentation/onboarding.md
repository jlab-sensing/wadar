# Onboarding

> __Note:__  
> README last updated August 2025

> __Principal Investigator:__
> Colleen Josephson, cojoseph at ucsc dot edu

> __Maintainers:__
> Eric Vetha, ericdvet at gmail dot com
> Jake Lee, jlee211 at ucsc dot edu
> Ava Darbonne, adarbonn at ucsc dot edu

Welcome to the WaDAR project. This document serves to onboard you to the components of this project. This is a living document, and you are encouraged to update it as you navigate and improve this project.



## Tutorial

This subsection's purpose is to teach you to do the most basic task required in this project: capturing a radar scan. Prior to all steps, ensure you have cloned the latest version of this repository.

### Communication

This first step is to communicate with the radar using your personal laptop. You will need a UNIX operating system. We have a lab macbook that can be borrowed if you do not have a UNIX device, but setting up a UNIX partition of your choice on your personal laptop would be beneficial in the long-run.

**IMPORTANT: DO NOT UNPLUG THE RADAR WITHOUT RUNNING *init 0* TO SHUT THE RADAR DOWN. ABRUPTLY REMOVING THE RADAR'S POWER CAN RESULT IN THE EMMC GETTING CORRUPTED.**

1. Connect your computer to the BeagleBone Black that the radar is mounted on. After approximately one minute has passed, run:
```bash
ssh root@192.168.7.2
```
You are now ssh'd into the radar. From the radar, attempt to ssh back into your personal computer:
```bash
ssh insert_your_username@192.168.7.1
```

If this part worked correctly, it should have prompted you for your password. We would like to avoid that, since the data capture scripts would require the constant re-entry of passwords for each scan.

2. In order to avoid the password requirement, you must set up SSH keys. The basic steps from https://www.strongdm.com/blog/ssh-passwordless-login can be followed for the most part.

__Debugging Tips__
- Adding `PubkeyAcceptedAlgorithms +ssh-rsa` to `/etc/ssh/ssh_config` when the SSH key isn't being accepted.
- Call `sudo systemctl restart ssh` to restart the SSH server after the config file is changed.
- Validate the setup by SSHing into the radar and SSHing back to our device without a password prompt.
- If you are on an Apple product, you will likely need to manually start an SSH server. To do this:
    1. Open "System Settings" > "General" > "Sharing".
    2. If disabled, enable "Remote Login".
    - Note that you will also need to use "192.168.6.1/2" rather than "192.168.7.1/2" with `ssh`.

### First scan

The next step is to take the first scan.

1. While ssh'd into the radar, navigate to the FrameLogger directory:
```bash
cd FlatEarth/Demos/Common/FrameLogger
```

Now check the contents of stage1.json, which holds settings that the radar uses to capture data:
```
cat stage1.json
```

We want it to read:
```json
[
  {"DACMin" : 3800},
  {"DACMax" : 4900},
  {"DACStep" : 8},
  {"Iterations" : 16},
  {"PulsesPerStep" : 8},
  {"FrameStitch" : 1}
]
```

If it does not, first copy the current stage1.json to back it up:
```bash
cp stage1.json stage1-archive.json
```
Now update the json with the correct parameters.

2. With the settings confirmed to be correct, you are now ready to take your first scan. In MATLAB, navigate to the directory with the matlab scripts:
```bash
cd '/your/path/to/wadar/01_dsp/matlab'
```
You can also use the Open button as it will automatically take you to the correct directory. Make sure a data folder exists in the matlab directory.

Now, in the MATLAB terminal, run the CaptureData function:
```bash
CaptureDataset("tutorial", "data", 3, 2000, 200)
```
This takes 3 captures of 2000 frames at 200 frames per second. You should see 3 new files named tutorial*.frames, tutorial*.md5, and captureSettings.

3. Let's see what these scans look like. In matlab, open TestFrames.mlx and change localDataPath to *'/data/'* and captureName to *'/tutorial1.frames'*. Run this script and you will
see what the raw and DDC processed frames look like. Feel free to play around with this and see what the data looks like while the radar is pointed at different objects. In the FFT, you will see that there is no apparent peak. Redo the scan with the backscatter tags, and you will see a noticeable peak in the index corresponding to the tag's frequency. If you are working on this project on the backscatter, you should pause here and do these scans with the backscatter tags. Otherwise, you may skip that step.

4. Congratulations, you have completed the most essential step, interfacing and scanning with the radar. Your next steps will differ based on your role in the project. Before you find your next task, it's time to safely shut the radar down. While ssh'd into the radar, in the bash terminal run:
```bash
init 0
```

The radar is now safely shut down. You can confirm by seeing the lights go out. You may now unplug the radar from your personal laptop.

## Project roles

### DSP Engineer

The codebases found in *01_dsp* pertain to the digital signal processing code. This is primarily post-processing of collected radar data. If you working on the testing and prototyping of DSP or ML algorithms, this is your entry point. If you are working on the deployment of algorithms on the radar for real-time sensing, you can find live-processing software in *02_uwb/FlatEarth*.

### Microwave Engineer

The hardware of the radar can be found in *02_uwb/chipotle*. The hardware of the backscatter tag can be found in *03_backscatter_tag*. If you working on the PCB hardware of these RF components, these are your entry points.