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
#include "helpers_ui.h"

igl::opengl::glfw::imgui::ImGuiMenu menu;

using namespace er;

void initialize_visualizer_ui(igl::opengl::glfw::Viewer &viewer)
{
	// Attach a menu plugin
	viewer.plugins.push_back(&menu);

	// Add content to the default menu window
	menu.callback_draw_viewer_window = [&] () {
		if (app_state::get().show_igl_viewer_menu)
			menu.draw_viewer_menu();
	};

	menu.callback_draw_custom_window = [&] () {
		if (ImGui::BeginMainMenuBar()) {
			if (ImGui::BeginMenu("File")) {
				if (ImGui::MenuItem("Open", "Ctrl+O")) {
					std::string fname = igl::file_dialog_open();

					if (fname.length() == 0)
						return;

					std::cout << "Load file " << fname << std::endl;
					app_state::get().set_current_file(fname);
				}

				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Edit")) {
				if (ImGui::MenuItem("Undo", "CTRL+Z")) {}
				if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {}  // Disabled item
				ImGui::Separator();
				if (ImGui::MenuItem("Cut", "CTRL+X")) {}
				if (ImGui::MenuItem("Copy", "CTRL+C")) {}
				if (ImGui::MenuItem("Paste", "CTRL+V")) {}
				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Debug")) {
				ImGui::MenuItem("Console", NULL, &app_state::get().show_app_console);
				ImGui::MenuItem("Log", NULL, &app_state::get().show_app_log);
				ImGui::EndMenu();
			}

			ImGui::EndMainMenuBar();
		}

		ShowAppConsole(&app_state::get().show_app_console);
		ShowAppLog(&app_state::get().show_app_log);

		static const int flags = ImGuiWindowFlags_AlwaysAutoResize;
		{
			ImGuiIO& io = ImGui::GetIO();

			{
				ImGui::Begin("System", nullptr, flags | ImGuiWindowFlags_MenuBar);

				ImGui::Text("Playback");
				ImGui::SameLine(70); ImGui::Text("%s",
					app_state::get().capture_folder.c_str());

				if (ImGui::CollapsingHeader("Capture", ImGuiTreeNodeFlags_DefaultOpen)) {
					float w = ImGui::GetContentRegionAvailWidth();
					float p = ImGui::GetStyle().FramePadding.x;
					if (ImGui::Button("Load##Analysis", ImVec2((w - p) / 2.f, 0))) {
						std::string fname = igl::file_dialog_open();

						if (fname.length() == 0)
							return;

						std::cout << "Load file " << fname << std::endl;
						app_state::get().set_current_file(fname);
					}
					ImGui::SameLine(0, p);
					if (ImGui::Button("Save##Analysis", ImVec2((w - p) / 2.f, 0))) {

					}
				}

				ImGui::End();
			}

			//ImGui::GetIO().IniFilename = "interface.ini";

			{
				ImGui::Begin("Raw RGBD View", nullptr, flags);

				ImGui::Text("Color Spaces");
				ImGui::Checkbox("Show nvdi", &app_state::get().bool_tint_nvdi);
				ImGui::SliderFloat("NVDI", &app_state::get().cur_nvdi,
					app_state::get().min_nvdi,
					app_state::get().max_nvdi);

				ImGui::Checkbox("Show ir", &app_state::get().bool_tint_ir);
				ImGui::SliderFloat("IR", &app_state::get().cur_ir, app_state::get().min_ir, app_state::get().max_ir);
				//ImGui::SliderFloat("HSV", &cur_nvdi, min_nvdi, max_nvdi);

				ImGui::Text("Clipping Z");
				ImGui::SliderFloat("Min Z", &app_state::get().cur_min_clip[2], app_state::get().min_clip[2], app_state::get().max_clip[2]);
				ImGui::SliderFloat("Max Z", &app_state::get().cur_max_clip[2], app_state::get().min_clip[2], app_state::get().max_clip[2]);

				ImGui::Text("Clipping Y");
				ImGui::SliderFloat("clipping Y", &app_state::get().cur_max_clip[1], app_state::get().min_clip[1], app_state::get().max_clip[1]);

				ImGui::Text("Point Scale");
				ImGui::SliderFloat("scale", &app_state::get().point_scale, 0.001f, 0.02f);

				if (ImGui::Checkbox("Show IR data", &app_state::get().show_ir_only_data)) {
					app_state::get().invalidate_ui = true;
					printf(" Show IR data \n");
				};

				ImGui::Separator();
				if (app_state::get().playing)
					ImGui::Checkbox("Playing", &app_state::get().playing);
				else
					ImGui::Checkbox("Pause", &app_state::get().playing);

				ImGui::Checkbox("Debug", &app_state::get().bool_debug_verbose);
				ImGui::End();
			}

			{
				ImGui::Begin("Debug Ground", nullptr, flags);
				ImGui::Text("Rotation");
				ImGui::Checkbox("Override", &app_state::get().bool_override_rotation);
				ImGui::SliderFloat("RotX", &app_state::get().rot_x, -2 * M_PI, 2 * M_PI);
				ImGui::SliderFloat("RotY", &app_state::get().rot_y, -2 * M_PI, 2 * M_PI);
				ImGui::SliderFloat("RotZ", &app_state::get().rot_z, -2 * M_PI, 2 * M_PI);

				ImGui::Text("Degree");
				ImGui::SameLine(100); ImGui::Text("%2.2f", app_state::get().rot_x * 360 / (2 * M_PI));
				ImGui::SameLine(150); ImGui::Text("%2.2f", app_state::get().rot_y * 360 / (2 * M_PI));
				ImGui::SameLine(200); ImGui::Text("%2.2f", app_state::get().rot_z * 360 / (2 * M_PI));

				ImGui::Checkbox("Traslate", &app_state::get().bool_traslate);
				ImGui::End();
			}

			{
				ImGui::Begin("Plants View", nullptr, flags);

				ImGui::Text("Clipping Z");
				ImGui::SliderFloat("Min Z", &app_state::get().plant_min_Z, -0.1f, 3);
				ImGui::SliderFloat("Max Z", &app_state::get().plant_max_Z, -0.1f, 3);

				ImGui::Text("Clipping Y");
				ImGui::SliderFloat("Min Y", &app_state::get().plant_min_Y, -0.1f, 1);
				ImGui::SliderFloat("Max Y", &app_state::get().plant_max_Y, -0.1f, 1);

				ImGui::End();
			}
		}

		{
			ImGui::Begin("Analysis", &app_state::get().show_analysis, flags);

			if (ImGui::Checkbox("Show ground", &app_state::get().show_ground)) {
				app_state::get().invalidate_ui = true;
				printf(" Show ground\n");
			}

			if (ImGui::Checkbox("Show plants", &app_state::get().show_plants)) {
				app_state::get().invalidate_ui = true;
				printf(" Show plants\n");
			}

			if (ImGui::Checkbox("Extract plants", &app_state::get().bool_extract_plants)) {
				app_state::get().invalidate_ui = true;
				printf(" Extract plants\n");
			};

			ImGui::Separator();

			if (ImGui::Checkbox("BBX", &app_state::get().show_bbx)) {
				app_state::get().invalidate_ui = true;
				printf(" Color cluster \n");
			};

			if (ImGui::Checkbox("Raw Cloud", &app_state::get().bool_cloud_raw)) {
				app_state::get().invalidate_ui = true;
				printf(" Color cluster \n");
			};

			if (ImGui::Checkbox("Color cluster", &app_state::get().bool_color_cluster)) {
				app_state::get().invalidate_ui = true;
				printf(" Color cluster \n");
			};

			if (ImGui::Checkbox("Voxel Process", &app_state::get().bool_voxel_process)) {
				app_state::get().invalidate_ui = true;
				printf(" Voxel Process \n");
			};

			if (ImGui::Checkbox("Distance Filter", &app_state::get().bool_distance_filter)) {
				app_state::get().invalidate_ui = true;
				printf(" Distance filter \n");
			};

			if (ImGui::Checkbox("Ground plane", &app_state::get().show_ground_plane)) {
				app_state::get().invalidate_ui = true;
				printf(" Show ground plane \n");
			};

			if (ImGui::Checkbox("Ground alignment", &app_state::get().ground_alignment)) {
				app_state::get().invalidate_ui = true;
				printf(" Align ground \n");
			};

			/*
			if (ImGui::Checkbox("Ground alignment X", &app_state::get().ground_alignment_x)) {
				app_state::get().invalidate_ui = true;
				printf(" Align ground X \n");
			};

			if (ImGui::Checkbox("Ground alignment Y", &app_state::get().ground_alignment_y)) {
				app_state::get().invalidate_ui = true;
				printf(" Align ground Y \n");
			};
			*/

			if (ImGui::Checkbox("Show floor", &app_state::get().show_floor)) {
				app_state::get().invalidate_ui = true;
				printf(" Show floor \n");
			};

			ImGui::Separator();
			ImGui::Text("-- Cloud --");
			//ImGui::Text("Raw %d", cloud_raw->points.size());
			//ImGui::Text("Process %d", cloud->points.size());

			ImGui::End();
		}

	};
}