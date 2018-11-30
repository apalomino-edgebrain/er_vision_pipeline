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
// Filter to use clusters to find plants
//#############################################################################

#ifndef plant_separation_filter_H_
#define plant_separation_filter_H_

#include "../er-pipeline.h"
#include "../algebra/plane.h"

// This class does the individual plant separation so we can display each
// plant independently and start doing calculations on them.
//
// It generates a 2D map field of the current view and creates a list of plants
//

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#ifdef USE_PCL_1_8_0
#include <pcl/ml/kmeans.h>
#endif

#include "ground_filter.h"
#include "plant_definition.h"
#include "plant_extraction.h"

#ifdef USE_IMGUI
#include <imgui.h>
#endif

namespace er {
	class plants_separation_filter: public process_unit
	{
	public:
		pcl::PointXYZRGBA min_pt, max_pt;

		plants_separation_filter() {};
		~plants_separation_filter() {};

		bool process() override;
		void invalidate_view() override;

		// Render UI code
		std::mutex plants_mutex;
		std::vector<plant_definition> plants;

		void render_ui() override;
	};
}

#endif