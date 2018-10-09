//#############################################################################
                                 /*-:::--.`
                            -+shmMMNmmddmmNMNmho/.
                 `yyo++osddMms:                  `/yNNy-
              yo    +mMy:                       `./dMMdyssssso-
              oy  -dMy.                     `-+ssso:.`:mMy`   .ys
                ho+MN:                  `:osso/.         oMm-   +h
                +Mmd-           `/syo-                     :MNhs`
                `NM-.hs`      :syo:                          sMh
                oMh   :ho``/yy+.                             `MM.
                hM+    `yNN/`                                 dM+
                dM/  -sy/`/ho`                                hMo
                hMo/ho.     :yy-                             dM/
            :dNM/             :yy:                         yMy
            sy`:MN.              `+ys-                     +Mm`
            oy`   :NM+                  .+ys/`           `hMd.ys
            /sssssyNMm:                   `:sys:`     `oNN+   m-
                        .sNMh+.                   `:sNMdyysssssy:
                        -odMNhs+:-.`    `.-/oydMNh+.
                            `-+shdNMMMMMMMNmdyo/.
                                    `````*/
//#############################################################################
// Visualizer
//
// Pushes 3d Objects to the libIGL for drawing.
//#############################################################################

#ifndef VISUALIZER3D_H_
#define VISUALIZER3D_H_

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/lockable_adapter.hpp>
#include <mutex>
#include <iostream>
#include <random>

#include <stdio.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include "er-pipeline.h"

#include "application_state.h"

//-----------------------------------------------------------------------------
// Thread and helpers to render the cloud after processing.

namespace er {

	// Lockable object that contains our raw point cloud
	class FrameData : public boost::basic_lockable_adapter<boost::mutex>
	{
	public:
		bool initialized;
		uint32_t time_t;

		boost::mutex mtx;

		volatile uint8_t idx;
		volatile bool invalidate;

		pcl_ptr cloud;

		FrameData();

		void invalidate_cloud(pcl_ptr cloud_);
	};

	class worker_t
	{
	public:
		bool initialized = false;
		boost::thread *bthread;

	private:
		FrameData data;

		int    n_;
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;
		Eigen::MatrixXd C;

	public:
		// Launches the visualizer, creates a window and returns.
		void setup();

		// Adds an axis helper.
		// Since we cannot draw simple lines (wtf) on libigl
		// we will have to add some geometry with the ground plane and the axis

		void add_axis();

		// Converts the PCL cloud into a libIGL compatible mesh
		void compute_cloud();

		// Launches the thread
		void start();

		// Thread safe function to update the worker with a cloud
		void push_cloud(pcl_ptr cloud);
	};
}

//-----------------------------------------------------------------------------
// Launches the thread and the visualizer
er::worker_t *launch_visualizer();

#endif
//#############################################################################