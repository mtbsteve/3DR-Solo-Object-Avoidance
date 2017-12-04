# 3DR-Solo-Object-Avoidance

Overview
Note: this code is for experimental use only and constantly evolving!

Link to video:
https://youtu.be/co4E9P2Q5RQ

Requirements:
- Lidar Lightware SF/LW20 with serial connection
- Servo to control the scanning
- Arduino Mega (we require 2 UARTS)
- USB cable to connect the Arduino to the breakout board
- Solo accessory bay breakout board
- 3DR Solo running Arducopter3.5 on the Pixhawk or any 3DR Firmware version
- Latest Solex version for speech output

Prerequisites:
- Arduino:
  - install the libraries for the lightware SF20 https://github.com/LightWare-Optoelectronics/LW20-Api
  - deploy the Arduino sketch with the latest Arduino IDE
  
- Solo:
  - Bridge the 3DRID pin on the breakout board to GND to set the Solo IMX as USB host and then connect the Arduino to the USB connector on the breakout board.
  - You must have PyMata version 2.1 installed on Solo. Newer versions than 2.1 do not properly install on Solo. To download PyMata, go here: https://github.com/MrYsLab/pymata-aio

  - Then install the buttonmanager.py and shotmanager.py files in the Solo /usr/bin directory using a SSH client

The code works on all Solo firmware versions, however the LED visualization only works in Arducopter 3.5 and higher or with the special firmware of Hugh Eaves (thank you!) https://github.com/hugheaves/solo-led-control

Note: the code is not yet compatible with OpenSolo shotmanager!

Functionality:
The code is constantly evolving. Current features include:
- 3 sector scanning for obstacles
- scan is enabled by log-button press on the controller paddle. A short button press toggles the GoPro on/off as before
- the LEDs indicate the detection of an obstacle by flashing red (Left LED: obstacle to the left; right LED: obstacle to the right; both front LEDs: obstacle in center)
- If a front collision is detected, Solo goes into BRAKE mode in GPS assisted modes as well as in Zipline, MPCC, RTL and FollowMe smartshots. You can exit BRAKE by pressing the FLY button anytime
- Solo keeps track of the forward pitch angle. As soon as the pitch angle would cause the lidar vbeam to hit ground, obstace avoidance is temporarily disabled. Both front LEDs flash in yellow to indicate that status.
- Text and speech output in Solex 

Known limitations:
- no support for OpenSolo yet
- LED support only with AC3.5 and higher firmware
- Works reliably only under clear weather conditions. In rain, snow and fog, the SF20 reports randomly false readings. 

Feedback, issue reports and suggestions always welcome!
