# crazy-practical
Crazyflie hardware practical for the Aerial Robots course at EPFL.

In this practical, you will learn how to program a [Crazyflie](https://www.bitcraze.io/) to find and precisely land on a platform with the help of minimal sensory information. You will work in a team of maximum 4 members. Teams will be defined at the beginning of the course.

<p align="center">
<img width=400 src="https://github.com/dronecourse-epfl/crazy-practical/blob/master/docs/pictures/crazypractical_02.png"/>
</p>

The drone takes off from a fixed position placed on the cow-pattern carpet made of 4 by 4 pieces. Thanks to the optic-flow sensor and a z-range finder it can track its movements and find the landing pad on the carpet. Once that the position of the landing pad is correctly estimated, the drone has to precisely land on it.

To complete the practical, follow carefully the instructions below. They will guide you through the assembling of the hardware, the installation of the required software and the algorithm coding.

The second to last day of the practical, on Mon 25th May, every team will be given the coordinates of where to place the box on the matrass and by the end of the practical, on Tue 26th May, you will submit a video of your drone performing the task with a close-up of the final landing. The grading will be done accoring to the following criteria.

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
3. One additional [lipo battery](https://store.bitcraze.io/collections/spare-parts-crazyflie-2-0/products/240mah-lipo-battery-including-500ma-usb-charger)
4. One [flow deck v2](https://store.bitcraze.io/collections/decks/products/flow-deck-v2)
5. One cow-pattern carpet (4 by 4 pieces)

## 2. Assembling
Assembling your Crazyflie 2.X will probably take less than 10 minutes, but there are a few pitfalls. So make sure to follow the instructions [here](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#assembling).

## 3. Installing on a computer
Follow the instructions [here](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#inst-comp) to install the Crazyflie Client and connect to your Crazyflie.

## 3 bis. Setting up the radio interface
You need to make sure that you have the right usb permission for the radio interface. You can find extensive instructions [here](https://github.com/bitcraze/crazyflie-lib-python#setting-udev-permissions).
On your terminal issue the following commands:
```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
```
Then log out and log back in to update the plugdev group.

Create the following file
```
touch /etc/udev/rules.d/99-crazyradio.rules
```

Then edit it with the text editor of your preference, and add the following lines:
```
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
```


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
