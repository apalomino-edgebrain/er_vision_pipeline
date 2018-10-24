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

er::pipeline::pipeline()
{
    printf("+ Initialize pipeline\n");

	er::ground_filter *ground = new er::ground_filter();

	er::plants_filter *plants = new er::plants_filter();

	ground->f_callback_output = std::bind(&plants_filter::input, plants, std::placeholders::_1);
	plants->set_ground_filter(ground);

	process_units["ground"] = ground;
	process_units["plant_extraction"] = plants;
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

er::process_unit::process_unit() : f_callback_output { nullptr }
{

}

er::process_unit::~process_unit()
{

}

void er::process_unit::input(frame_2d type, void *color_frame)
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

	if (cloud_out == nullptr || view == nullptr)
		return;

	view->invalidate_cloud(cloud_out);
}

bool er::process_unit::process()
{
	return true;
}

void er::process_unit::input(pcl_ptr cloud)
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
	pu->input(cloud);

	// Initialize the display frame buffers

	// TODO: (this is not thread safe).
	// Maybe lock on the worker thread?
	for (auto const &x : process_units) {
		process_unit *pu = x.second;
		if (pu->view == nullptr) {
			pu->view = new er::frame_data();
			data_views.push_back(pu->view);
		}

		// Here we do the render pre-process
		if (pu->visible) {
			pu->invalidate_view();
		}
	}
}
