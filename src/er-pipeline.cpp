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
//
// See er-pipeline.h to get more information on the functionality and structure
//

#include "er-pipeline.h"

#include "filters/ground_filter.h"
#include "filters/plant_extraction.h"
#include "filters/plant_separation.h"
#include "filters/plant_segmentation.h"

//-----------------------------------------------------------------------------
// STD
//-----------------------------------------------------------------------------

#include <string>
using namespace std;

//-----------------------------------------------------------------------------
// BOOST System
//-----------------------------------------------------------------------------

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

// The current pipeline is as follows:
//
// 1. Ground separation filter
//    Finds what the ground is.
//	  Separates the ground points for rendering
//    Finds the plane in which the ground floor lives.
//
//	  Ground_filter Output -> Plants_filter Input
//
// 2. Plants filter process to segmentation
//	  Separates plants and removes points closer to ground which might not be important
//	  Cleans data
//
//	  Plants_filter Output -> Plants_Separation Input
//
// 3. Finds the individual plants
//    Creates a list of plants for current frame with positions
//    2D images
//
er::pipeline::pipeline()
{
    printf("+ Initialize pipeline\n");

	// Build a factory for this
	er::ground_filter *ground = new er::ground_filter();
	er::plants_filter *plants = new er::plants_filter();
	er::plants_separation_filter *plants_extract = new er::plants_separation_filter();
	er::plants_segmentation_filter *plants_seg = new er::plants_segmentation_filter();

	ground->f_callback_output =
		std::bind(&er::plants_filter::input_pcl, plants, std::placeholders::_1);

	plants->set_ground_filter(ground);
	plants->f_callback_output =
		std::bind(&er::plants_separation_filter::input_pcl, plants_seg, std::placeholders::_1);

	process_units["ground"] = ground;
	process_units["plant_segmentation"] = plants;
	//process_units["plants_separation"] = plants_extract;
	process_units["plants_seg"] = plants_seg;
}

er::pipeline::~pipeline()
{
	// TODO: Delete the process_units
}

void er::pipeline::preconfigure_process_units(std::string process_units_path)
{

}

void er::pipeline::initialize_folder(std::string folder_path)
{
	/*
    path data_path(folder_path);
    if (!is_directory(data_path)) {
        data_path = data_path.parent_path();

        if (!is_directory(data_path)) {
            printf("! Error: Not a directory [%s]\n", folder_path.c_str());
        }
    } else {
        printf("+ Processing [%s]\n", folder_path.c_str());
    }

    for (directory_iterator itr(data_path); itr != directory_iterator(); ++itr) {
        // display filename only
        cout << itr->path().filename() << ' ';
        if (is_regular_file(itr->status()))
            cout << " [" << file_size(itr->path()) << ']';

        cout << '\n';
    }
	*/
}

//-----------------------------------------------------------------------------
// process_units system
//-----------------------------------------------------------------------------

void er::process_unit::start_process()
{
	using namespace std::chrono;
	t_start = high_resolution_clock::now();
}

void er::process_unit::end_process()
{
	using namespace std::chrono;
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	t_elapsed = duration_cast<duration<double>>(t2 - t_start);
}

er::process_unit::process_unit() : f_callback_output { nullptr }
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// This is our cloud to be reused after processing
	cloud_out = cloud;
}

er::process_unit::~process_unit()
{

}

void er::process_unit::input_frame(frame_2d type, void *color_frame)
{
	// Check if the process was able to finish with this new information
	if (!process()) {
		return;
	};
}

void er::process_unit::invalidate_view()
{
	if (!visible)
		return;

	if (cloud_out == nullptr || view.visible == false)
		return;

	view.invalidate_cloud(cloud_out);
}

bool er::process_unit::process()
{
	return true;
}

void er::process_unit::input_pcl(pcl_ptr cloud)
{
	cloud_in = cloud;

	// Check if the process was able to finish with this new information
	if (!process()) {
		return;
	};
}

pcl_ptr er::process_unit::output(int id)
{
	return nullptr;
}

//-----------------------------------------------------------------------------
// Main processing for our input frame
//-----------------------------------------------------------------------------

// We will also add the OpenCV images.

// When we process a frame we get our cached frame data
// We have different views over the data so we can display all kinds of intermediate
// processed objects.

void er::pipeline::process_frame(pcl_ptr cloud, std::vector<frame_data *> &data_views)
{
	process_unit *pu = process_units["ground"];
	pu->input_pcl(cloud);

	// Prepare the list of views to render, only visible views will get into the
	// rendering pipeline

	bool subscribe = false;
	if (data_views.size() == 0) {
		subscribe = true;
	}

	for (auto const &x : process_units) {
		process_unit *pu = x.second;

		// Here we do the render pre-process
		if (pu->visible) {
			pu->invalidate_view();
			if (subscribe)
				data_views.push_back(&pu->view);
		}
	}
}

// Call to render our UIs, each process unit has the ability to render it's
// own window on the UI. This call has to be done from the UI thread.

void er::pipeline::render_ui()
{
	for (auto const &x : process_units) {
		process_unit *pu = x.second;
		pu->render_ui();
	}
}