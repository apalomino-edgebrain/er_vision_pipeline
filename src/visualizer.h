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
		bool update_view = false;
		boost::thread *bthread;

	private:
		std::vector<frame_data *> data_views;

	public:
		// Encapsulate the vector with the views. We will refactor all this.
		std::vector<er::frame_data *> &get_data_views();

		// Launches the visualizer, creates a window and returns.
		void setup();

		// Adds an axis helper.
		// Since we cannot draw simple lines (wtf) on libigl
		// we will have to add some geometry with the ground plane and the axis

		void add_axis();

		void test_cube();

		// Converts the PCL cloud into a libIGL compatible mesh
		void compute_cloud();

		// Launches the thread
		void start();

		int axis_index;
	};
}

//-----------------------------------------------------------------------------
// Launches the thread and the visualizer
er::worker_t *launch_visualizer();

#endif
//#############################################################################