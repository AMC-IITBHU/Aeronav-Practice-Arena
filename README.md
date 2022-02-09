# Aeronav-Practice-Arena
This repository consists of the Practice Arena, for the event  Aeronav'22 being conducted by the Aeromodelling Club, IIT-BHU, Varanasi.


## Installation instructions
In the autonomous event we are using Webots simulation.

![img1](webots.png)

To visit Webots offical site [click here](https://cyberbotics.com/)

###   Webots installation
- Download Webots installation file from the links given below:
  - [Windows](https://github.com/cyberbotics/webots/releases/download/R2022a/webots-R2022a_setup.exe)
  - [macOS](https://github.com/cyberbotics/webots/releases/download/R2022a/webots-R2022a.dmg)
  - [linux (Ubuntu 20.04)](https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb)
  
  (For other versions of operating system visit Webots official site.)

### Python requirement
-  Teams are supposed to use default python version in their OS.
-  Modules and Libraries which they want to use should be installed on their default python.

### World setup

Just clone this git repo and extract the task.zip file.

There will be a world file (.wbt) in this zip file . Teams have to open this world file in Webots and work on it.

>NOTE:- For any doubts regarding any of the above instruction refer [this link](https://cyberbotics.com/doc/guide/index)

## Task for the event
At the beginning of the simulation, your drone must be standing atop the starting platform with its stands in contact with it.
You will devise a function on the controller that will take input 2 different RGB values(RGB1 and RGB2) at the start which you’ll input on being provided with the colours of our choosing while the drone rests atop the starting platform.Only after manually receiving the 2 RGB values, should the drone take-off from the pad. The task for the event is divided into following subtasks:
- Fly the drone above a specific height to follow the road and use the camera underneath to count number of boxes and balls of colors **RGB1 and RGB2** respectively.
- After counting the no. of relevant boxes and obtaining values of RGB1 and RGB2, caluculate value of X using equation:
  - _**X = | 2 x RGB1 – RGB2 |**_
- At the end of the track scan all the QR codes each of which contains 3 values; and id and x,y coordinates of the QR.
 ![image](https://user-images.githubusercontent.com/82452505/153142152-0f7978d1-75bb-478a-8f10-8691a0ffaaee.png)
- The drone has to be landed on the QR with **id = X**, which was the solution of the equation.


The complete problem statement can be found [here](https://github.com/AMC-IITBHU/Aeronav-Practice-Arena/blob/main/AeroNav%202K22.pdf).

## Drone Specifications:
- We are using `Dji Mavic 2 pro`, which have a downward facing camera to take images.

- We have provided basic code with some function and a basic but not perfect control system from drone . Teams could either use these or use their own code. (The provided code is basically for your understanding about how to code in controller).
- To know about the specification of drone refer this [link](https://www.cyberbotics.com/doc/guide/mavic-2-pro?version=develop#mavic2pro-field-summary)

## Submission instructions
Further instruction on submitting the solution will be given soon.
