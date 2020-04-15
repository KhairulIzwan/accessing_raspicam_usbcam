# accessing_raspicam_usbcam

1. Learn about the RPi camera module and USB webcams
2. Pick a camera and get started with it
3. Learn about the VideoStream class and how it can access either a USB webcam 
or RPi camera module using OpenCV

## script
1. **access_camera.py**: Basic way of accessing camera; however its not suit
enough for ROS implementation
2. **camera_preview.py**: Accessing camera (usb) in ROS way
3. **raspicam_preview.py**: Accessing camera (raspicam) in ROS way

## launch
1. **accessing_raspicam.launch**: Accessing camera (usb) in ROS way
2. **access_camera.launch**: Accessing camera (raspicam) in ROS way

> Required ROS packages
>> 1. raspicam
>>> https://github.com/UbiquityRobotics/raspicam_node
>> 2. vc_camera
>>> https://github.com/OTL/cv_camera
