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

namespace er {
    class app_state
    {
    public:
		app_state();
        ~app_state();

        // Loads the application configuration.
		// Here we specify if we have a frontend or not.

        void load_configuration(std::string json_configuration_file);

		// Returns a singleton containing the application state.
		static app_state &get()
		{
			static er::app_state instance;
			return instance;
		}

		//-----------------------------------------------
		// UI API

		bool show_app = true;

		bool show_analysis = true;

		bool show_ground = true;
		bool show_plants = true;
		bool show_ir_only_data = true;
		bool show_ground_plane = false;

		bool bool_cloud_raw = false;
		bool bool_color_cluster = false;
		bool bool_distance_filter = true;
		bool bool_voxel_process = false;

		bool bool_tint_nvdi = true;
		bool bool_tint_ir = true;

		float min_nvdi = 1;
		float cur_nvdi = -1;
		float max_nvdi = -1;

		float min_ir = 0;
		float cur_ir = 0;
		float max_ir = 1;

		float cur_max_clip[3] = { 0, 10, 10 };
		float cur_min_clip[3] = { 0, 0, 0 };

		float min_clip[3] = { 0, 0, 50 };
		float max_clip[3] = { 0, 50, -50 };

		//-----------------------------------------------
		float point_scale = 0.01f;

		//-----------------------------------------------
		bool playing = true;
		bool bool_extract_plants = false;
    };

}

#endif