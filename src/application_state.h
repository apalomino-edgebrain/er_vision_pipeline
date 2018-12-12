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

#ifndef application_state_H_
#define application_state_H_

#define DEFAULT_CONFIG "vision_cfg.json"
#define DEFAULT_DATASET_CONFIG "dataset.json"

#include <Eigen/Core>

using namespace nlohmann;

namespace er {
	class app_state
	{
	public:
		std::string config_path;

		json config;

		app_state();
		~app_state();

		// Returns a singleton containing the application state.
		static app_state &get()
		{
			static er::app_state instance;
			return instance;
		}

		//-----------------------------------------------
		// We store data per dataset so we can browse in an organized way.
		//

		std::string dataset_root;
		std::string dataset_config_path;
		json json_dataset;

		// Our current dataset is a large JSON file which contains every
		// folder we processed with information regarding the data analysed and
		// notes written by classification

		void load_dataset(std::string json_config);

		void save_current_dataset();

		// We save a sort description of what the data is in a file
		// so we can classify files easily.
		void save_description(std::string description);

		//-----------------------------------------------
		// Loads the application configuration.
		// Here we specify if we have a frontend or not.

		std::string json_configuration_file_path_;

		void load_configuration(std::string json_configuration_file_path_);

		// Specify a folder to save the config.
		// We might want to read specific configs for different data sets.

		void save_configuration(std::string json_configuration_file_path_);

		// Save the current config into the main folder
		void save_configuration();

		bool invalidate_playback = false;

		//-----------------------------------------------
		// Flush all the data that we want to store from processing this
		// including notes about the data.
		void populate_data_file();

		// Full path to the folder containing the dataset with JSON files
		// Example: X:\data\8799ecf\180904_123201
		std::string capture_folder;

		// Full path to the current bag being played
		// Example: X:\data\8799ecf\180904_123201\capture.bag
		std::string capture_bag;

		// Just the ID of the file
		// 180904_123201
		std::string capture_id;

		// Current data options
		json json_data;

		std::string get_bagfile_from_id(std::string folder_id);
		void set_current_file(std::string filepath_playback);

		//-----------------------------------------------
		// UI API

		bool show_plant_ui_window = true;

		bool show_igl_viewer_menu = false;
		bool show_app_console = false;
		bool show_app_log = false;

		bool bool_debug_verbose = false;

		bool invalidate_ui = false;

		bool show_plants_raw = false;
		bool show_ground_raw = false;

		bool show_ground = false;
		bool show_plants = true;
		bool show_ir_only_data = true;
		bool show_ground_plane = false;
		bool show_floor = true;

		bool show_bbx = true;

		bool bool_cloud_raw = false;
		bool bool_color_cluster = false;
		bool bool_distance_filter = true;
		bool bool_voxel_process = false;

		bool bool_tint_nvdi = true;
		bool bool_tint_ir = true;

		// Aligns the ground with our unit Axis
		bool bool_traslate = true;

		bool ground_alignment = true;
		bool ground_alignment_x = true;
		bool ground_alignment_y = true;

		float min_nvdi = 1;
		float cur_nvdi = -0.343;
		float max_nvdi = -1;

		float min_ir = 0;
		float cur_ir = 0.361f;
		float max_ir = 1;

		float cur_max_clip[3] = { 0, 10, 10 };
		float cur_min_clip[3] = { 0, 0, 0 };

		float min_clip[3] = { 0, 0, 50 };
		float max_clip[3] = { 0, 50, -50 };

		//-----------------------------------------------
		bool show_voxel_view = true;

		float leaf_X = 0.01f;
		float leaf_Y = 0.01f;
		float leaf_Z = 0.01f;

		float scale_voxel_grid = 0.01f;

		// Euclidian Cluster algorithms
		bool show_euclidian_cluster = true;
		int min_cluster_points = 90;
		int max_cluster_points = 10000;
		float cluster_tolerance = 0.02; // 2cm

		bool show_voxel_data = false;

		// Kmeans cluestering
		bool show_kmeans_cluster = true;
		int cluster_size = 16;

		bool use_voxel_grid = false;
		bool use_octreepoint_voxel = true;

		//-----------------------------------------------
		bool use_erosion = false;
		float erosion_resolution = 0.5f;

		//-----------------------------------------------
		// Window views

		bool show_camera_window = true;
		bool show_debug_ground = true;
		bool show_raw_rgbd = true;
		bool show_system_view = true;
		bool show_plants_view = true;
		bool show_app = true;
		bool show_analysis = true;

		//-----------------------------------------------
		// Plants distantances

		float plant_min_Y = 0.05f;
		float plant_max_Y = 0.50f;     // Max 1 meter

		float plant_min_Z = 0;
		float plant_max_Z = 3;

		//-----------------------------------------------
		float point_scale = 0.01f;

		bool bool_override_rotation = false;
		float rot_x = 0.0f;
		float rot_y = 0.0f;
		float rot_z = 0.0f;

		//-----------------------------------------------
		// We reproject the floor into a texture 2.5d map

		bool show_image_view = true;

		float floor_project_x = 0;
		float floor_project_y = 0;

		//-----------------------------------------------
		bool playing = true;
		bool bool_extract_plants = false;

		void config_to_json();
		void json_to_config();

		void save_vec3f(const char *name, Eigen::Vector3f &vec);
		Eigen::Vector3f load_vec3f(const char *name);

		void save_vec3d(const char *name, Eigen::Vector3d &vec);
		Eigen::Vector3d load_vec3d(const char *name);

		void save_vec4f(const char *name, Eigen::Vector4f &vec);
		Eigen::Vector4f load_vec4f(const char *name);

		//-----------------------------------------------
		uint64_t playback_position;
		uint64_t playback_duration;

		//-----------------------------------------------
		// This is our exit trigger. We close the system if we detect
		// that this is set to true.
		bool should_close_app = false;
	};

}

#endif