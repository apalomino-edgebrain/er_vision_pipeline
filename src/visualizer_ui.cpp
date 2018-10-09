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
// Imgui for the visualizer
//#############################################################################

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/lockable_adapter.hpp>
#include <mutex>
#include <iostream>
#include <random>

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include <igl/jet.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include "er-pipeline.h"

#include "application_state.h"

igl::opengl::glfw::imgui::ImGuiMenu menu;

void initialize_visualizer_ui(igl::opengl::glfw::Viewer &viewer)
{
	// Attach a menu plugin
	viewer.plugins.push_back(&menu);

	// Customize the menu
	float floatVariable = 0.1f; // Shared between two menus

	// Add content to the default menu window
	menu.callback_draw_viewer_menu = [&] () {
		// Draw parent menu content
		menu.draw_viewer_menu();

		/*
		// Add new group
		if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen)) {
			// Expose variable directly ...
			ImGui::InputFloat("float", &floatVariable, 0, 0, 3);

			// ... or using a custom callback
			static bool boolVariable = true;
			if (ImGui::Checkbox("bool", &boolVariable)) {
				// do something
				std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
			}

			// Expose an enumeration type
			enum Orientation { Up = 0, Down, Left, Right };
			static Orientation dir = Up;
			ImGui::Combo("Direction", (int *) (&dir), "Up\0Down\0Left\0Right\0\0");

			// We can also use a std::vector<std::string> defined dynamically
			static int num_choices = 3;
			static std::vector<std::string> choices;
			static int idx_choice = 0;
			if (ImGui::InputInt("Num letters", &num_choices)) {
				num_choices = std::max(1, std::min(26, num_choices));
			}
			if (num_choices != (int) choices.size()) {
				choices.resize(num_choices);
				for (int i = 0; i < num_choices; ++i)
					choices[i] = std::string(1, 'A' + i);
				if (idx_choice >= num_choices)
					idx_choice = num_choices - 1;
			}
			ImGui::Combo("Letter", &idx_choice, choices);

			// Add a button
			if (ImGui::Button("Print Hello", ImVec2(-1, 0))) {
				std::cout << "Hello\n";
			}
		}
		*/
	};

	menu.callback_draw_custom_window = [&] () {
		static const int flags = ImGuiWindowFlags_AlwaysAutoResize;
		{
			ImGuiIO& io = ImGui::GetIO();
			//ImGui::GetIO().IniFilename = "interface.ini";

			ImGui::Begin("app", nullptr, flags);
			ImGui::Text("Color Spaces");
			ImGui::Checkbox("Show nvdi", &er::app_state::get().bool_tint_nvdi);
			ImGui::SliderFloat("NVDI", &er::app_state::get().cur_nvdi,
										er::app_state::get().min_nvdi,
										er::app_state::get().max_nvdi);

			ImGui::Checkbox("Show ir", &er::app_state::get().bool_tint_ir);
			ImGui::SliderFloat("IR", &er::app_state::get().cur_ir, er::app_state::get().min_ir, er::app_state::get().max_ir);
			//ImGui::SliderFloat("HSV", &cur_nvdi, min_nvdi, max_nvdi);

			ImGui::Text("Clipping Z");
			ImGui::SliderFloat("Min Z", &er::app_state::get().cur_min_clip[2], er::app_state::get().min_clip[2], er::app_state::get().max_clip[2]);
			ImGui::SliderFloat("Max Z", &er::app_state::get().cur_max_clip[2], er::app_state::get().min_clip[2], er::app_state::get().max_clip[2]);

			ImGui::Text("Clipping Y");
			ImGui::SliderFloat("clipping Y", &er::app_state::get().cur_max_clip[1], er::app_state::get().min_clip[1], er::app_state::get().max_clip[1]);

			ImGui::Text("Point Scale");
			ImGui::SliderFloat("scale", &er::app_state::get().point_scale, 0.005f, 0.2f);
			ImGui::End();
		}

		{
			ImGui::Begin("Analysis", &er::app_state::get().show_analysis, flags);
			ImGui::Checkbox("Show ground", &er::app_state::get().show_ground);
			ImGui::Checkbox("Show plants", &er::app_state::get().show_plants);
			ImGui::Checkbox("Extract plants", &er::app_state::get().bool_extract_plants);
			ImGui::Separator();

			ImGui::Checkbox("Raw Cloud", &er::app_state::get().bool_cloud_raw);
			ImGui::Checkbox("Color cluster", &er::app_state::get().bool_color_cluster);
			ImGui::Checkbox("Voxel Process", &er::app_state::get().bool_voxel_process);
			ImGui::Checkbox("Distance Filter", &er::app_state::get().bool_distance_filter);

			ImGui::Separator();
			if (er::app_state::get().playing)
				ImGui::Checkbox("Playing", &er::app_state::get().playing);
			else
				ImGui::Checkbox("Pause", &er::app_state::get().playing);

			ImGui::Separator();
			ImGui::Text("-- Cloud --");
			//ImGui::Text("Raw %d", cloud_raw->points.size());
			//ImGui::Text("Process %d", cloud->points.size());

			ImGui::End();
		}

	};
}