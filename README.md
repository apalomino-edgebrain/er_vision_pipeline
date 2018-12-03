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

# ER Vision Pipeline

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