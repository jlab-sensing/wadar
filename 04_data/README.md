# WADAR Radar Captures

# Radar Captures

This directory contains directions on how to access the server required containing radar captures taken by the Chipotle UWB. Follow the instructions below to access and transfer files. 

If you are not added to the HARE-Lab data server, please use the backup repository at [wadar-database](https://github.com/jlab-sensing/wadar-database). It is not added as a submodule here because of its size.

SSH Access
----------

1. To SSH into the server:

   Use the following command to SSH into the server where the radar captures are stored:

   ssh [USERNAME]@harelab-files.harelab.soe.ucsc.edu

   - Replace [USERNAME] with your username.

2. Navigate to the radar captures directory:

   Once logged in, use the following command to navigate to the directory where the radar captures are stored:

   cd /storage/nas/jlab/WaDAR/CaptureBackups

SCP Access (for file transfers)
-------------------------------

To transfer files between your local machine and the server, you can use scp (Secure Copy Protocol). Below are the basic commands for uploading and downloading files.

1. To upload files to the server:

   From your local machine, use the following scp command to upload files to the server:

   scp /path/to/local/file [USERNAME]@harelab-files.harelab.soe.ucsc.edu:/storage/nas/jlab/WaDAR/CaptureBackups

   - Replace /path/to/local/file with the location of the file on your local machine.
   - Replace [USERNAME] with your relevant details.
   - Use -r before the file if the file being transferred is a folder.

2. To download files from the server:

   Use this command to copy files from the server to your local machine:

   scp [USERNAME]@harelab-files.harelab.soe.ucsc.edu:/storage/nas/jlab/WaDAR/CaptureBackups /path/to/local/destination/

   - Replace /path/to/local/destination/ with the location on your local machine where you want to save the file.
   - Use -r before the file if the file being transferred is a folder

Mounting a Local Directory with SSHFS
-------------------------------------

To access files from your local PC as if they were on the server, you can use SSHFS to mount a directory from your local machine to the server. This allows seamless access to local files.

Steps to Mount:

1. Install SSHFS (if not already installed):
     sudo apt-get install sshfs

2. Mount your local directory on the server:

   TODO: fill this out

Troubleshooting
---------------

- Ensure that you have the correct permissions to access the directory.
- If you encounter connection issues, verify the server address and your SSH keys or password.
