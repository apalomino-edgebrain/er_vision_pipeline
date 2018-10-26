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
// Application global state
//#############################################################################

#include <string>
#include <iostream>

#include "json/json.hpp"
using namespace nlohmann;

#include "application_state.h"
#include "util/er-logging.h"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

er::app_state::app_state()
{
	config["compilation"] = __DATE__ __TIME__;
	printf("+ Initialize pipeline\n");
	load_configuration(DEFAULT_CONFIG);
}

er::app_state::~app_state()
{

}

void er::app_state::json_to_config()
{
	show_plant_ui_window = config["show_plant_ui_window"];

	show_igl_viewer_menu = config["show_igl_viewer_menu"];
	show_app_console = config["show_app_console"];
	show_app_log = config["show_app_log"];

	bool_debug_verbose = config["bool_debug_verbose"];

	invalidate_ui = config["invalidate_ui"];

	show_app = config["show_app"];

	show_analysis = config["show_analysis"];

	show_ground = config["show_ground"];
	show_plants = config["show_plants"];
	show_ir_only_data = config["show_ir_only_data"];
	show_ground_plane = config["show_ground_plane"];
	show_floor = config["show_floor"];

	show_bbx = config["show_bbx"];

	bool_cloud_raw = config["bool_cloud_raw"];
	bool_color_cluster = config["bool_color_cluster"];
	bool_distance_filter = config["bool_distance_filter"];
	bool_voxel_process = config["bool_voxel_process"];

	bool_tint_nvdi = config["bool_tint_nvdi"];
	bool_tint_ir = config["bool_tint_ir"];

	// Aligns the ground with our unit Axis
	bool_traslate = config["bool_traslate"];

	ground_alignment = config["ground_alignment"];
	ground_alignment_x = config["ground_alignment_x"];
	ground_alignment_y = config["ground_alignment_y"];

	cur_nvdi = config["cur_nvdi"];
	cur_ir = config["cur_ir"];

	plant_min_Y = config["plant_min_Y"];
	plant_max_Y = config["plant_max_Y"];

	plant_min_Z = config["plant_min_Z"];
	plant_max_Z = config["plant_max_Z"];

	point_scale = config["point_scale"];
	bool_override_rotation = config["bool_override_rotation"];

	bool_extract_plants = config["bool_extract_plants"];
}

void er::app_state::config_to_json()
{
	config["show_plant_ui_window"] = show_plant_ui_window;

	config["show_igl_viewer_menu"] = show_igl_viewer_menu;
	config["show_app_console"] = show_app_console;
	config["show_app_log"] = show_app_log;

	config["bool_debug_verbose"] = bool_debug_verbose;

	config["invalidate_ui"] = invalidate_ui;

	config["show_app"] = show_app;

	config["show_analysis"] = show_analysis;

	config["show_ground"] = show_ground;
	config["show_plants"] = show_plants;
	config["show_ir_only_data"] = show_ir_only_data;
	config["show_ground_plane"] = show_ground_plane;
	config["show_floor"] = show_floor;

	config["show_bbx"] = show_bbx;

	config["bool_cloud_raw"] = bool_cloud_raw;
	config["bool_color_cluster"] = bool_color_cluster;
	config["bool_distance_filter"] = bool_distance_filter;
	config["bool_voxel_process"] = bool_voxel_process;

	config["bool_tint_nvdi"] = bool_tint_nvdi;
	config["bool_tint_ir"] = bool_tint_ir;

	// Aligns the ground with our unit Axis
	config["bool_traslate"] = bool_traslate;

	config["ground_alignment"] = ground_alignment;
	config["ground_alignment_x"] = ground_alignment_x;
	config["ground_alignment_y"] = ground_alignment_y;

	config["cur_nvdi"] = cur_nvdi;
	config["cur_ir"] = cur_ir;

	config["plant_min_Y"] = plant_min_Y;
	config["plant_max_Y"] = plant_max_Y;

	config["plant_min_Z"] = plant_min_Z;
	config["plant_max_Z"] = plant_max_Z;

	config["point_scale"] = point_scale;
	config["bool_override_rotation"] = bool_override_rotation;

	config["bool_extract_plants"] = bool_extract_plants;
}

void er::app_state::save_vec3f(const char *name, Eigen::Vector3f &vec)
{
	config[name] = { vec(0), vec(1), vec(2) };
}

Eigen::Vector3f er::app_state::load_vec3f(const char *name)
{
	Eigen::Vector3f out;
	try {
		out[0] = config[name][0];
		out[1] = config[name][1];
		out[2] = config[name][2];
	} catch (const std::exception & e) {

	}
	return out;
}

void er::app_state::save_vec3d(const char *name, Eigen::Vector3d &vec)
{
	// Todo find how to do this conversion
	Eigen::Vector3f v = Eigen::Vector3f(vec.x(), vec.y(), vec.z());
	save_vec3f(name, v);
}

Eigen::Vector3d er::app_state::load_vec3d(const char *name)
{
	Eigen::Vector3f v = load_vec3f(name);
	Eigen::Vector3d vec = Eigen::Vector3d(v.x(), v.y(), v.z());
	return vec;
}

void er::app_state::save_vec4f(const char *name, Eigen::Vector4f &vec)
{
	config[name] = { vec(0), vec(1), vec(2), vec(3) };
}

Eigen::Vector4f er::app_state::load_vec4f(const char *name)
{
	Eigen::Vector4f out;
	try {
		out[0] = config[name][0];
		out[1] = config[name][1];
		out[2] = config[name][2];
		out[3] = config[name][3];
	} catch (const std::exception & e) {
		printf("Exception");
	}
	return out;
}

void er::app_state::load_configuration(std::string json_config)
{
	try {
		config_path = json_config;
		printf_("Reading config", json_config.c_str());
		std::ifstream i(json_config.c_str());
		i >> config;
	} catch (const std::exception & e) {
		// No config? No worries, we create a new one.
		config["version"] = __DATE__ __TIME__;
		save_configuration();
	}

	printf_h2("Config");

	std::cout << std::setw(4) << config << std::endl;
}

void er::app_state::save_configuration()
{
	config_to_json();
	std::ofstream o(config_path.c_str());
	o << config.dump(4);

	std::cout << "Save" << std::endl;
}

void er::app_state::set_current_file(std::string filepath_playback)
{
	path data_path(filepath_playback);
	if (!is_directory(data_path)) {
		capture_bag = filepath_playback;
		capture_folder = data_path.parent_path().string();
	} else {
		capture_bag = filepath_playback + "/capture.bag";
		capture_folder = filepath_playback;
	}

	invalidate_playback;
	save_configuration();
}
