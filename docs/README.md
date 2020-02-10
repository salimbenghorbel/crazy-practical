# crazy-practical
Crazyflie hardware practical for the Aerial Robots course at EPFL.

In this practical, you will learn how to program a [Crazyflie](https://www.bitcraze.io/) to precisely land on a platform with the help of minimal sensory information. You will work in a team of maximum 4 members. Teams will be defined at the beginning of the course.

<p align="center">
<img width=400 src="https://github.com/dronecourse-epfl/crazy-practical/blob/master/docs/pictures/crazypractical_02.png"/>
</p>

**TODO: update image here!**

To complete the practical, follow carefully the instructions below. They will guide you through the assembling of the hardware, the installation of the required software and the algorithm coding.

At the end of the practical, on Tue 26th May, we will organize a competition during the course hours where your algorithm will compete. The grading will be done accoring to the following criteria:

## Grading

Your final score will be made up of four parts. Three of the scores are binary, so you either fullfill them, in which case you get all points, or you do not, which means you receive no points. 

Binary scores, each worth 5 points:
* _s_: started - the Crazyfie took off successfully
* _a_: alive - there was no crash during the run (Crazyflie landed without crash or is still hovering at the end of the task)
* _r_: reached target - the Crazyflie touched the top surface of the platform

On top of that, there is a continuous score which is worth 10 points. It will grade you on the time needed to finish the task. This score will only be assigned if you fullfill _r_ (i.e. if the Crazyflie touched the top surface of the platform). If you need the whole 2 minutes, you will receive zero points. If you need zero seconds, you will receive 10 points. The scores in between will be distributed linearly.

Continuous score, worth max 10 points:
* _t_ = (2min - time_needed)/(2min) * 10points

The final score will therefore be:

_score_ = _s_ + _a_ + _r_ + _t_

**TODO: add link on moodle pointing here for more info about scoring system**

## Requirements
For this practical, each team is required to use one of their personal laptops. Bitcraze supports the installation of the software on Windows, Linux, OS X and Virtual Machines. However, we tested the installation process only on:
1. Windows 10
2. Ubuntu 16.04

## 1. Unpacking
At the beginning of the practical, every team will receive a box with the necessary hardware. This includes:
1. One [Crazyflie package](https://store.bitcraze.io/collections/kits/products/crazyflie-2-1)
  * 1 x Crazyflie 2.1 control board with all components mounted
  * 5 x CW propellers
  * 5 x CCW propellers
  * 6 x Motor mounts
  * 1 x LiPo battery (240mAh)
  * 5 x Coreless DC motors
  * 2 x Short expansion connector pins (1×10, 2mm spacing, 8 mm long)
  * 2 x Long expansion connector pins (1×10, 2mm spacing, 14 mm long)
  * 1 x Battery holder expansion board
  * 1 x USB cable (only with the Crazyflie 2.1)
2. One [spare part bundle](https://store.bitcraze.io/collections/spare-parts-crazyflie-2-0/products/spare-part-bundle)
  * 2 x spare motors
  * 2 spare motor mounts pack
  * spare propellers (4 x CW and 4 x CCW)
4. One additional [lipo battery](https://store.bitcraze.io/collections/spare-parts-crazyflie-2-0/products/240mah-lipo-battery-including-500ma-usb-charger)
3. One [flow deck v2](https://store.bitcraze.io/collections/decks/products/flow-deck-v2)
4. One pair of safety glasses for every team member
5. One cow-pattern carpet

## 2. Assembling
Assembling your Crazyflie 2.X will probably take less than 10 minutes, but there are a few pitfalls. So make sure to follow the instructions [here](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#assembling).

## 3. Installing on a computer
Follow the instructions [here](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#inst-comp) to install the Crazyflie Client and connect to your Crazyflie.

## 4. Configuring the Crazyradio and the Crazyflie address
Every team will be assigned a unique radio channel and Crazyflie address. All you need to do is:
* Plug the Crazyradio in your pc
* Start the Crazyflie Client
* Select your Crazyradio and 'Scan'
* Connect to your Crazyfie (default address is 0xE7E7E7E7E7)
* In the 'Connect' tab on top, select 'Configure 2.X'
* In the window that openned up, enter the channel number and radio address that have been assigned to your team. Then, click 'Write'.
* Restart the Crazyflie and the client
* Connect to your Crazyfie using your assigned address

## 5. Cloning the repo
For developing the code for the practical, you will need to clone this repository with its submodules via the following commands:

```git clone https://github.com/dronecourse-epfl/crazy-practical/```

```git submodule update --init --recursive```

# Coding your algorithm
Now it's your time to code!
You will find example scripts for this practical in `crazyflie-lib-python/cp_example`.

## Reading and saving sensor measurements
## Analysing logs
## Sending commands
