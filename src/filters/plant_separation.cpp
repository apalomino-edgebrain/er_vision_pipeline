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
// Plant extraction process_unit
//#############################################################################
//
// See er-pipeline.h to get more information on the functionality and structure
//

#include <limits>

#include "../er-pipeline.h"
#include "../application_state.h"
#include <igl/fit_plane.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include "ground_filter.h"
#include "plant_extraction.h"
#include "plant_separation.h"

using namespace er;

//------ Plant discovery & segmentation ------

bool plants_separation_filter::process()
{
	cloud_out->clear();

	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(app_state::get().leaf_X,
		app_state::get().leaf_Y, app_state::get().leaf_Z);
	sor.filter(*cloud_out);
	return true;
}

void plants_separation_filter::invalidate_view()
{
	if (view != nullptr)
		view->point_scale = er::app_state::get().scale_voxel_grid;
	er::process_unit::invalidate_view();
}