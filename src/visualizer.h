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

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/jet.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include "er-pipeline.h"

#include "application_state.h"

//-----------------------------------------------------------------------------
// Launches the thread and the visualizer
void launch_visualizer();

// Appends the UI to the viewer
void initialize_visualizer_ui(igl::opengl::glfw::Viewer &viewer);

//-----------------------------------------------------------------------------
// Thread and helpers to render the cloud after processing.

namespace er {
	class worker_t
	{
	private:

		int    n_;
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;
		Eigen::MatrixXd C;

	public:
		void setup(size_t n);

		// Adds an axis helper.
		// Since we cannot draw simple lines (wtf) on libigl
		// we will have to add some geometry with the ground plane and the axis

		void add_axis();

		// Converts the PCL cloud into a libIGL compatible mesh
		void compute_cloud(igl::opengl::glfw::Viewer &viewer);

		// Launches the visualizer, creates a window and returns.
		void start(int n);

		// Used to launch the thread;
		void operator()();
		boost::thread *bthread;
	};
}

#endif
//#############################################################################