EARTH ROVER PIPELINE
====================

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
====================

The idea for the pipeline is that we have filters in a similar structure to 
a common video pipeline but using a mixture between 3D and 2D.

From the Realsense we get several buffers that we pass through a set of 
filters in realtime or not to get into the required results. 

This pipeline should work with 2D, 3D, OpenCV, Realsense, CNNs... 
It is up to the filter to understand the data and return something meaninful 
to the next item on the pipeline.

Filters accept several inputs of any type. After the filter is happy
with all the inputs it will return true and let the next filter catch the results.

This code should work and run in the following ways
	* Realtime ROS Node 
		Running on a low spec device in a ROS environment publishing outputs.
		All the code running on this filter should be production quality.

	* Research & Development UI
		Development of filters and tools for video processing

	* Offline batch process
		Generating valuable data by analysing objects and plants

	* Classification UI
		Providing the tools to classify data in 2D and 3D for the integrated 
		neural network

## Development Philosophy

We value results over code quality at this stage. 
Code has to be functional and readable, with plenty of links to documentation
and information related on how an algorithm or integration works.

The project has to be simple to compile in Linux, Embedded and the Windows platform.

## Pipeline

```

//#############################################################################
// Pipeline Philosophy
//
// 1. Requirements
//
// Our pipeline is based on the input from a realsense data and the extra
// metadata captured by our capturing system.
//
// This API is experimental and we are still defining the functionality and
// and roadmap.
//
// On our requirements list we have:
//
// - Offline - UI less processing of data, generating JSON outputs that can be
// digested by a Geo located database
//
// - Color and Infrared processing
// - Hyperspectral processing
// - Geo located object data
// - Time located object data
//
// - Farm specific data
//	 . Plant
//	 . Row
//   . Field
//
//	 . Analyse
//		Growth
//		Health
//
// 2. Specifications
//
// The pipeline is a heavily multithreaded system with processing units.
//
// A processing unit is our minimum interface that is able to process an input.
// Given that our different inputs are asynchronous, the processing unit has
// to evaluate when all the requirements are met to generate a result.
//
// A processing unit can callback many units waiting for this result.
// An example of this could be the HSV color space conversion which might be
// required by several units.
//
// 3. System architecture
//
// Jobs can be spawned on the system with priorities, every unit should try
// to block as less as possible the UI. Therefore the last process stage
// is just a PCL dump into the visualizer or disk.
//

```

## Process Unit

The process unit is our base for our architecture. 
The pipeline calls a factory to build the filters specified by the configuration (To be done)

This factory creates and links all the filters or process units.

er-pipeline.h contains a description how a process unit works

```
	// We have process_units on the system that accept point clouds, images
	// and metadata
	//
	// This process unit is able to generate an output and callback a function
	// with the result.
	//
	// The process_units can contain a tree of dependencies and can be multithreaded.
	class process_unit
```

## Inputs and Outputs

Filters have inputs and outputs. This will be expanded later on with the API.

## Ground Filter

### Floor Discovery and Ground extraction

```
//------ BASIC FLOOR DISCOVERY & MAPPING ------
// Create floor grid
//  Width x Height

// Algorithm of floor removal and floor discovery
// Split in two clusters
//		1. Some green + and nvdi = Plant
//		2. No color, just ground
//
// Find the plane defined by the ground plane
//
// Adjust the plane to fit the current view using the Minimun coefficients
// Find the angle X to align the plane on that axis
// Find the angle Y to align the plane on that axis
//
// Create a transformation matrix to fit orientate the plane
// Transfor every position to fit the new axis base.

// Next steps:
//	Reduce floor cloud density with a voxelgrid
//  Raytrace points from floor grid up to find the right position in space
//  Classify between floor and plants

```

## Architecture

* [LibRealsense](https://github.com/IntelRealSense/librealsense)
	Intel's 3D capturing hardware

* [LibIgl](http://libigl.github.io)
	Library for geometrical processing, we use it for render 

* [LibPCL](http://pointclouds.org/)
	Point Cloud Library 

* [Imgui](https://github.com/ocornut/imgui)
	Immediate User Interface 

* OpenCV & Deep neural network
	Computer Vision library
	[DNN](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/opencv/dnn)	

* [PointNet](https://arxiv.org/abs/1612.00593)