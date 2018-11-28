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
// Filter to use raw algorithms on the data to find the plants.
//#############################################################################

#ifndef plant_segmentation_filter_H_
#define plant_segmentation_filter_H_

#include "../er-pipeline.h"
#include "../algebra/plane.h"

// This class does the individual plant segmentation so we can display each
// plant independently and start doing calculations on them.
//
// It generates a 2D map field of the current view and creates a list of plants
//

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/ml/kmeans.h>

#include "ground_filter.h"
#include "plant_definition.h"
#include "plant_extraction.h"

#ifdef USE_IMGUI
#include <imgui.h>
#include "examples\opengl2_example\imgui_impl_glfw_gl2.h"
#endif

// Fast and dirty implementation for a 2.5d image analisis
// to find the different plants.

#define FLOOR_SIZE_W 64
#define FLOOR_SIZE_H 64

namespace er {
	class plants_segmentation_filter: public process_unit
	{
	public:
		// Texture to display
		uint32_t size_w;
		uint32_t size_h;
		unsigned int tex_id;

		uint32_t *tex_floor_rgba;
		float *floor_projection;

		pcl::PointXYZRGBA min_pt, max_pt;

		plants_segmentation_filter();
		~plants_segmentation_filter();

		bool process() override;
		void invalidate_view() override;

		// Render UI code
		std::mutex plants_mutex;
		std::vector<plant_definition> plants;

		void render_ui() override;
	};
}

#endif