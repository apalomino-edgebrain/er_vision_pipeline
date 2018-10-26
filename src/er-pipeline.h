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
// Pipeline to process our PointCloud data
//#############################################################################

#ifndef pipeline_H_
#define pipeline_H_

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

//-----------------------------------------------------------------------------
// SYSTEM
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/lockable_adapter.hpp>
#include <mutex>
#include <iostream>
#include <random>

//-----------------------------------------------------------------------------
// LIBREALSENSE
//-----------------------------------------------------------------------------

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

//-----------------------------------------------------------------------------
// JSON processing
//-----------------------------------------------------------------------------

// Examples https://github.com/nlohmann/json#examples
#include "json/json.hpp"

#define PCL_NO_PRECOMPILE

//-----------------------------------------------------------------------------
// PCL System
//-----------------------------------------------------------------------------

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

#include <pcl/point_types.h>

#include "impl/region_growing.hpp"
#include "impl/region_growing_rgb.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr;

//-----------------------------------------------------------------------------
// TOOLS
//-----------------------------------------------------------------------------

#include "util/er-logging.h"

typedef Eigen::Transform<double, 3, Eigen::Affine> Tform3;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Quaterniond Quat;

//-----------------------------------------------------------------------------
// FARM PIPELINE
//-----------------------------------------------------------------------------

#include "process_3d.h"

#include <mutex>
#include <ctime>
#include <ratio>
#include <chrono>

struct plane;

namespace er {
	enum class frame_2d
	{
		color, ir, ir2, depth, region
	};

	// Lockable object that contains our raw point cloud
	// And is able to render to our opengl views
	class frame_data
	{
	public:
		// Do we render this view on the screen?
		volatile bool visible;

		Eigen::Vector3d bbx_m;
		Eigen::Vector3d bbx_M;

		bool initialized;
		uint32_t time_t;

		std::mutex mtx;

		volatile uint32_t idx;
		volatile bool invalidate;

		pcl_ptr cloud;

		frame_data();

		// Pushes a cloud into this item
		void invalidate_cloud(pcl_ptr cloud_);

		// Helper to render text
		void render_text(void *viewer_ptr, Eigen::RowVector3d &cp,
			const char *text);

		// Helper to render a plane
		void render_plane(void *viewer_ptr,
			plane &ground_plane,
			Eigen::Vector3d &m,
			Eigen::Vector3d &M);

		// Renders a point in space and text
		void render_point(void *viewer_ptr,
			Eigen::RowVector3d &cp,
			Eigen::Vector3d &color,
			const char *text = nullptr);

		//----------
		// Renderer

		volatile int id_mesh;
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;
		Eigen::MatrixXd C;
		Eigen::VectorXd radius;

		void calculate_view();
		void render(void *viewer);

		// Callback in case we want to have a custom renderer for this data frame
		std::function<void(void *viewer)> f_external_render = nullptr;
	};

	// We have process_units on the system that accept point clouds, images
	// and metadata
	//
	// This process unit is able to generate an output and callback a function
	// with the result.
	//
	// The process_units can contain a tree of dependencies and can be multithreaded.
	class process_unit
	{
	public:
		std::chrono::high_resolution_clock::time_point t_start;
		std::chrono::duration<double> t_elapsed;

		void start_process();
		void end_process();

		// Each unit should be able to calculate the time it took to process.
		// TODO Benchmarking on the process units

		// Do we show this unit on the display?
		bool visible = true;

		frame_data *view = nullptr;

		pcl_ptr cloud_in;
		pcl_ptr cloud_out;

		process_unit();
		~process_unit();

		void input_frame(frame_2d type, void *color_frame);
		void input_pcl(pcl_ptr cloud);

		// Process the current process_unit and runs the algorithms
		// Returns true if the process has finished so we call the callback
		virtual bool process();

		// Sends the cloud out to frame data if we are looking to render it
		// Units can render whatever they want by overriding this call
		virtual void invalidate_view();

		// Callback function in case there is someone registered to it.
		// This might end up being a vector of callbacks since we will
		// have multiple processes waiting for this result.
		std::function<void(pcl_ptr)> f_callback_output;

		pcl_ptr output(int id);
	};

    class pipeline
    {
    public:
		std::map<std::string, process_unit *> process_units;

        pipeline();
        ~pipeline();

        // Preconfigures a folder where the information for the video is stored.
        // Preprocess all the information on that folder and caches the frame data
        // in memory.
        void initialize_folder(std::string folder_path);

		// Load process_units
		// process_units are algorithmic functions that are able to process point clouds
		// asynchronously and can register to different inputs, either OpenCV, RGB, Sensor data, etc.
		void preconfigure_process_units(std::string process_units_path);

		void process_frame(pcl_ptr cloud, std::vector<frame_data *> &data_views);
    };
}

#endif