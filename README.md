# UR3e-HandTracking-Control
Program to control one or two UR3e robots using a Leap Motion hand tracking camera.

- This project could not have been completed without the help of:
  - Xiaomeng Shi, Assistant Teaching Professor at Widener University
  - Jack Merhar
  - Brennon Conner
  - Keanu Williams
  - Emily Wolfe

Required:
- Leap Motion Hand Tracking Camera
- Leap Motion Gemini SDK (https://developer.leapmotion.com/tracking-software-download)
- Numpy Python Module (https://pypi.org/project/numpy/)
- Quaternion Python Module (https://pypi.org/project/numpy-quaternion/)
- UR-RTDE Python Module (https://pypi.org/project/ur-rtde/)
- Robotiq Hand-E for gripper usage
- Universal Robots UR3e Robot(s) or the UR3e Linux simulator
- Visual Studio 2019 dlls required for running the LeapMotionC.exe program
  - ucrtbased.dll
  - vcruntime140d.dll 

Usage:
- Start UR3eControl.py program with correct Host PC IP Address and UR3e Robot IP Addresses.
- Start LeapMotionC.exe program with Leap Motion plugged into PC and Leap Motion Control Panel Running.
- Begin control of robotic arms by moving your hands to the virtual position of the robotic end effector positions and opening the palms.
- Close and open the Robotiq grippers by piching the thumb and index finger together.
- Stop control boy making a fist with each respective hand and moving hands away from the virtal robotic end effector positions.

Source:
- Source code for the LeapMotionC.exe program is provided as HandTrackingMain.c and can be edited and recompiled by linking the LeapMotionSDK into your coding environment.

Control System Diagram:

![Picture1](https://user-images.githubusercontent.com/86205659/178172862-f0dedfad-577e-4dbe-865a-4c49118a7408.png)
![Hand-Tracking-Layout](https://user-images.githubusercontent.com/86205659/178177196-c4f625a5-b8a3-4b78-a220-e01895038b16.PNG)
![Hand-Tracking-Gestures](https://user-images.githubusercontent.com/86205659/178177017-4134fdf8-2806-4964-8a4a-a6c762687cdf.PNG)

Hand Tracking Control Videos:

https://user-images.githubusercontent.com/86205659/178176459-bd42a7d8-7cd3-4ccf-934c-66d4922a1b5c.mp4

https://user-images.githubusercontent.com/86205659/178176042-18feb7fe-6183-40f0-9569-26e53960c8f0.mp4

https://user-images.githubusercontent.com/86205659/178176045-506b4128-b72e-4e9c-b818-36fe7b64203e.mp4

