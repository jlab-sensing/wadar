"""
This program is free software: you can redistribute it and/or modify it under the terms of the 
GNU
General Public License as published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without
even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.
The GNU General Public License can be found at <
http://www.gnu.org/licenses/
>.
"""
"""
Linux interface to Mini
-
Circuits matrix switch
RC
-
2SP6T
-
A12.
Written by: Tim Dunker
E
-
mail: tdu [at] justervesenet [dot] no
OpenPGP key: 0x4FDBC497
This programme is based on the script 'Minimalist Linux interface to Mini Circuits USB power 
meter', copyright (C) 2017, Rigetti & Co. Inc., distributed 
with a GPL licence. """

# Basic Usage
from MiniCircuits_Switch import MiniCircuits_Switch
sw = MiniCircuits_Switch()
import time

#Get device model number
modNum = sw.get_device_serial_number()
print(modNum)

#Reset Switches 
sw._query([9, 0])
switchStatus = sw._query([15])
print(switchStatus[1])

#Switch channels with timed while loop
startTime = time.time()
runT = 1 #seconds
endTime = startTime + runT
runCount = 0
#while (time.time() < endTime):
while (True):
    sw._query([9,0])
    #print(sw._query([15])[1])
    sw._query([9,1])
    #print(sw._query([15])[1])
    #time.sleep(.1)
    #print(time.time())
    #runCount += 1

print(runCount)

###Switch channels with for loop
##startTime = time.time()
##for i in range(0,100):
##    sw._query([9,1])
##    sw._query([9,0])
##
##tTotal = time.time()-startTime
##print(100/tTotal)

#Reset Switches 
sw._query([9, 0])

###De-energize switch A:
##sw.switch(1,0)
###De-energize switch B:
##sw.switch(2,0)
###De-energize switch C:
##sw.switch(3,0)
### De-energize switch D:
##sw.switch(4,0)

# Switch to channel 2 on switch A:
#switchStatus = sw._query([15])
#print(switchStatus[1])
#sw._query([9, 3])
#switchStatus = sw._query([15])
#print(switchStatus[1])
#sw._query([9, 3])
#switchStatus = sw._query([15])
#print(switchStatus[1])
# De-energize switch A:
#sw.switch(1,0)
# De-energize switch B:
#sw.switch(2,0)
# De-energize switch C:
#sw.switch(2,0)
# De-energize switch D:
#sw.switch(2,0)
#Check status 
#switchStatus = sw._query([15])
#print(switchStatus[1])

#Get device model number
#modNum = sw.get_device_serial_number()
#print(modNum)


