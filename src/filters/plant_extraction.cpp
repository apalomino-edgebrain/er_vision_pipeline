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

#include "ground_filter.h"
#include "plant_extraction.h"

using namespace er;

#define NVDI -0.345f
#define IR (0.361f * 255.f)
#define MAX_LUMINOSITY (150.0f*3.0f)

#define MAX_Z 3

//------ Plant discovery & Extraction ------
// After we align the floor

bool plants_filter::process()
{
	if (cloud_out == nullptr) {
		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		cloud_out = cloud;
	} else {
		// TODO: Do not clear and reuse points
		cloud_out->clear();
	}

	Eigen::RowVector3d cp;

	for (auto& p : cloud_in->points) {
		bool add_point = true;

		if (p.r + p.g + p.b > MAX_LUMINOSITY) {
			add_point = false;
		} else
		if (p.a > IR) {
			add_point = false;
		} else {
			float nvdi = float(p.a - p.r) / (p.a + p.r);

			if (nvdi > NVDI) {
				add_point = false;
			}
		}

		if (add_point && p.z >= 0.01f)
			cloud_out->points.push_back(p);
	}

	return true;
}

void plants_filter::set_ground_filter(ground_filter *grnd_filter_)
{
	grnd_filter = grnd_filter_;
}

void plants_filter::invalidate_view()
{
	if (!visible)
		return;

	if (cloud_out == nullptr || view == nullptr)
		return;
}
