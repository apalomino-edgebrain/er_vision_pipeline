//#############################################################################
// Pipeline
//
// We have an ingestion system that accepts processed data from the realsense
// and the different resources we capture.
//
//
// This system accepts
//		Geo information from JSON
//			Orientation
//			Lat and Long
//			State of the RTK and precision.
//
//		Raw images
//			Color
//			Infrared
//			Depth
//
//		Realsense Raw Point Cloud
//		PCL - Point Cloud Data
//
// This part of the system is able to extract information from the raw data
// We have a system of processing plugins which register to obtain access
// to the different resources
//
// This system has to be able to expose bottlenecks by adding timers and
// checking for processes that would not be able to run in realtime.
//
//#############################################################################

#ifndef PROCESS3D_H_
#define PROCESS3D_H_

#include "field_map.h"

#endif
//#############################################################################