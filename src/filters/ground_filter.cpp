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
// Ground extraction process_unit
//#############################################################################
//
// See er-pipeline.h to get more information on the functionality and structure
//

#include "../er-pipeline.h"
#include "../application_state.h"

#include "ground_filter.h"

using namespace er;

#define NVDI -0.345f

bool ground_filter::process()
{
	if (cloud_out == nullptr) {
		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		cloud_out = cloud;
	} else {
		// TODO: Do not clear and reuse points
		cloud_out->clear();
	}

	for (auto& p : cloud_in->points) {
		bool add_point = true;

		float nvdi = float(p.a - p.r) / (p.a + p.r);

		if (nvdi > NVDI) {
			add_point = false;
		}

		if (add_point && p.z >= 0.01f)
			cloud_out->points.push_back(p);
	}

	return true;
}

void ground_filter::invalidate_view()
{
	if (!visible)
		return;

	if (cloud_out == nullptr || view == nullptr)
		return;

	view->invalidate_cloud(cloud_out);
}
