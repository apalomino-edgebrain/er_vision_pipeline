INSTALLATION EARTH ROVER ROS VISION
===================================

```
                        `.-:::--.`                         
                   -+shmMMNmmddmmNMNmho/.                   
        `..`   `+hNMds/-.         `.:+ymMmy:                
     `yyo++osddMms:                     `/yNNy-             
     yo    +mMy:                         `./dMMdyssssso-    
     oy  -dMy.                     `-+ssso+:.`:mMy`   .ys   
      ho+MN:                  `:osso/.          oMm-   +h   
      `mMd`               ./sys/.                :NN: /d`   
      +Mmd-           `/syo-                      :MNhs`    
     `NM-.hs`      :syo:                           sMh      
     oMh   :ho``/yy+.                              `MM.     
     hM+    `yNN/`                                  dM+     
     dM/  -sy/`/ho`                                 hMo     
     hMo/ho.     :yy-                               dM/     
     +MM/          .oh/`                           .MM.     
    :dNM/             :yy:                         yMy      
   sy`:MN.              `+ys-                     +Mm`      
  oh   /MN-                .+ys-                 +MNy       
  oy`   :NM+                  .+ys/`           `hMd.ys      
   /sssssyNMm:                   `:sys:`     `oNN+   m-     
           -hMm+`                    `:oss+:sNNs`   `m:     
             .sNMh+.                   `:sNMdyysssssy:      
                -odMNhs+:-.`    `.-/oydMNh+.                
                   `-+shdNMMMMMMMNmdyo/.                    
                           `````                            
                            ``                              
      ydsssss`     hm`     ydssh/  `sssmdsss  +h      :d    
      yo          yhod`    ys  .M.     ho     +h      /m    
      yh+++:     od` sh    ys:oho      ho     +m++++++ym    
      ys...`    +m`   hy   ys.sh:      ho     +d......+m    
      yy::::-  :m.    `do  ys  .sh:    ho     +h      /m    
                                                            
```


## Introduction

This project can work in two modes:
1. Realtime, using a realsense camera

2. Offline, playing back a recording made on the field.

The code used at the moment to capture on the field is located here:
[er_camera_capture](https://github.com/earthrover/er_camera_capture).

# Librealsense Installation

## Getting started

### Cloning the repository

On the current repository we have all our dependencies in submodules.

GIT
----------------------------

```
sudo apt-get -y install git
```

GITHUB SETUP FOR DEVELOPERS
---------------------------

#### Setup your github credentials
###### [Github Article](https://help.github.com/articles/set-up-git/)

```
git config --global user.name "Bob the Minion"
git config --global user.email "bob@earthrover.cc"
git config --global core.editor vim

```

#### Setup your git certificate
###### [Github with SSH](https://help.github.com/articles/connecting-to-github-with-ssh/)

```
ssh-keygen -t rsa -b 4096 -C "bob@earthrover.cc"

```

#### Add the agent to our user boot

Add ssh agent to bashrc so it autostarts when a terminal is created
```
vim ~/.bashrc

```

Add to the bottom of the file
```
eval $(ssh-agent -s) > /dev/null

```

Add your credentials to ssh for the ssh client to find
```
eval $(ssh-agent -s) 
ssh-add ~/.ssh/id_rsa

```

#### Add SSH key to github
###### [Github add key to account](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/)

Add your SSH to github and test 

```
ssh git@github.com

```

Something like this should be your output
```
earth@earth-pi-ros:~/$ ssh git@github.com
PTY allocation request failed on channel 0
Hi sergioamr! You've successfully authenticated, but GitHub does not provide shell access.
Connection to github.com closed.
```


ROS
----------------------------

#### Install ROS and Build tools

If you have ROS installed you can skip this part:

###### [Installation](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Install Build essentials
------------------------

Prerequisites

## Install CMAKE 3.8+

Example on 3.13
```
mkdir cmake_download & cd cmake_download
wget https://github.com/Kitware/CMake/releases/download/v3.13.1/cmake-3.13.1.tar.gz
tar xvf cmake-3.13.1.tar.gz
cd cmake-3.13.1/
./configure
sudo make install
sudo hash -r
```

## Install ROS Essentials
```
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
sudo rosdep init
rosdep update
```

CLONE EARTH ROVER VISION REPO
-----------------------------

```
git clone git@github.com:earthrover/er_vision_pipeline.git
cd er_vision_pipeline
```

Init all our libraries and dependencies
```
git submodule init
```

Update and download the right versions of the libraries
```
git submodule update
```

## libigl Installation

LibIGl is a geometric processing library that enables point cloud analysis and 
```
cd libs/libigl/
mkdir build
cd build
cmake ..
make
sudo make install
```

## Realsense Installation

Before trying to install and run the code from this repository we have to install the realsense. Here is a more complete version on how to install the realsense.
For the Jetson you have to jump into the Jetson installation part.

[Librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md).

```
cd libs/librealsense
```

## Prepare Linux Backend and the Dev. Environment:
```
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev  -y
sudo apt-get install libglfw3-dev -y 
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
```

If everything went well:
```
sudo make uninstall && make clean && make && sudo make install
```

## Vision Pipeline
Now lets try to compile er_vision_pipeline

```
cd ~/er_vision_pipeline/
mkdir build & cd build
cmake ..
make
```

## How to run it

To run the project, the application will accept two types of input.
1. A bag file containing captured data by er_camera_capture
2. A realsense live input

If you downloaded our data set, you should have a folder like

```
~/data/8799ecf
```

8799ecf is the reference of the commit from er_camera_capture

Example of a good data sample:
```
er-vision ~/data/8799ecf/180904_135414/capture.bag
```

This installation will not give support to the IMU and some modes

## Jetson TX2 

We are usign as our main computing platform the Nvidia Jetson TX2 and the Astro Carrier RevG

Our first vision platform was a ZED camera. 

You can find code related to our data capturing here:
[ZED Camera capturing system](https://github.com/earthrover/earth-rover-ros-vision/blob/master/JETSON.md).
On that repo you will find help and know how on how to flash and clone our Jetson devices.

This was deprecated and we moved into the Realsense D435:

[Realsense on the jetson Tx2](https://github.com/jetsonhacks/buildLibrealsense2TX).

In order to be able to have support for the different components we need to install the Jetpack 3.2 and deploy our applications layer on top.

This repo contains the ROS code that will run at the Jetson and our main vision systems taking advantage of the computing platform.

### Utils - Force mode

We have a headless HDMI setup so we have to force the NVidia driver to change resolution into a virtual mode.

```
/etc/X11/xorg.conf
```

```
Section "Module"
    Disable     "dri"
    SubSection  "extmod"
        Option  "omit xfree86-dga"
    EndSubSection
EndSection

Section "Device"
    Identifier  "Tegra0"
    Driver      "nvidia"
# Allow X server to be started even if no display devices are connected.
    Option      "AllowEmptyInitialConfiguration" "true"
EndSection

Section "Monitor"
    Identifier     "Monitor0"
    VendorName     "VNC"
    ModelName      "VNC"
    HorizSync       15.0 - 70.0
    VertRefresh     58.0 - 62.0
    Option         "DPMS"
    Modeline       "1024x768_60.00"   63.50  1024 1072 1176 1328  768 771 775 798 -hsync +vsync
EndSection

Section "Screen"
    Identifier     "Screen0"
    Device         "Tegra0"
    Monitor        "Monitor0"
    DefaultDepth    24

    SubSection     "Display"
        Virtual     1024 768
    EndSubSection

EndSection
```
