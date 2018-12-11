EARTH ROVER ROS VISION
======================

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

LICENSE
===================

BSD 3-Clause License

Copyright (c) 2018, Earth Rover 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ER Vision Pipeline

![](https://github.com/earthrover/er_vision_pipeline/blob/e91479f08560cd10862ca5cb55423170f7a3e8ac/assets/plants_classifier.png)

This project is our image analyse for our data captured ROS bags.
OpenCV and PCL processing of our sample data

Check INSTALL.md to see how to setup the system

# System

We use the Realsense D435 camera as our main platform.

# Color experiments
Near-Infrared, or NIR, is the waveband ranging roughly from 700 to 1000nm

Our camera laser sits on:
Laser Wavelength 850nm ± 10 nm nominal @ 20°C

Left and Right Imager are able to see in the following wavelengths:

400 to 865 (Visible and Infrared) @ 98% transmission
rate or higher at all viewing and transmitting angles
# Example
![](https://github.com/earthrover/er_vision_pipeline/blob/e91479f08560cd10862ca5cb55423170f7a3e8ac/assets/image.png?raw=true)
