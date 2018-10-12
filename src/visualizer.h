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

#include "er-pipeline.h"

#include "application_state.h"

struct plane;

//-----------------------------------------------------------------------------
// Thread and helpers to render the cloud after processing.

namespace er {
	class worker_t
	{
	public:
		bool initialized = false;
		boost::thread *bthread;

	private:
		frame_data data;

		int    n_;
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;
		Eigen::MatrixXd C;

		uint32_t id_axis = -1;
		uint32_t id_plane = -1;
		uint32_t id_mesh = -1;

		er::pipeline pipeline;

	public:
		// Launches the visualizer, creates a window and returns.
		void setup();

		// Adds an axis helper.
		// Since we cannot draw simple lines (wtf) on libigl
		// we will have to add some geometry with the ground plane and the axis

		void add_axis();

		void test_cube();

		// Converts the PCL cloud into a libIGL compatible mesh
		void show_plane(plane &p, Eigen::Vector3d &m, Eigen::Vector3d M);

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