# crazy-practical
Crazyflie hardware practical for the Aerial Robots course at EPFL. During the crazy-practical you will work in a team of maximum 4 members. Teams will be defined at the beginning of the course.

## Requirements
For this practical, every team will one of their own laptop. Bitcraze supports the installation of the software on Windows, Linux, OS X and Virtual Machines. However, we tested the installation process only on:
1. Windows 10
2. Ubuntu 16.04

## 1. Unpacking
At the beginning of the practical, every team will receive a box with the necessary hardware. This includes:
1. One Crazyflie package
  * 1 x Crazyflie 2.X control board with all components mounted
  * 5 x CW propellers
  * 5 x CCW propellers
  * 6 x Motor mounts
  * 1 x LiPo battery (240mAh)
  * 5 x Coreless DC motors
  * 2 x Short expansion connector pins (1×10, 2mm spacing, 8 mm long)
  * 2 x Long expansion connector pins (1×10, 2mm spacing, 14 mm long)
  * 1 x Battery holder expansion board
  * 1 x USB cable (only with the Crazyflie 2.1)
2. One spare part bundle
  * 2 x spare motors
  * 2 spare motor mounts pack
  * spare propellers (4 x CW and 4 x CCW)
3. One flow deck v2
4. One pair of safety glasses for every team member
5. One cow-pattern carpet

## 2. Assembling
Assembling your Crazyflie 2.X will probably take less than 10 minutes, but there are a few pitfalls. So make sure to follow the instructions [here](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#assembling).

## 3. Installing on a computer
Follow the instructions [here](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#inst-comp) to install the Crazyflie Client and connect to your Crazyflie.

## 4. Configuring the Crazyradio and the Crazyflie address
Every team will be assigned a unique radio channel and Crazyflie address. All what you need to do is:
* plug the Crazyradio in your pc
* start the Crazyflie Client
* select your Crazyradio and 'Scan'
* connect to your Crazyfie (default address is 0xE7E7E7E7E7)
* in the 'Connect' tab on top enter the new Crazyradio channel and Crazyflie address
* restart the Craazyflie

## 5. Cloning the repo
For developing the code for the practical, you will need to clone this repo with the following command:
```git clone https://github.com/dronecourse-epfl/crazy-practical/```

# Coding your algorithm
Now it's your time to code!
You will find example scripts for this practical in `crazyflie-lib-python/cp_example`.

## Reading and saving sensor measurements
## Analysing logs
## Sending commands
